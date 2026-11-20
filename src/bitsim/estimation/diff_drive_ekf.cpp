#include "bitsim/estimation/diff_drive_ekf.h"
#include "bitsim/maths/math_utils.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace bitsim
{
namespace estimation
{

using maths::wrap_angle;

// ---------------------------------------------------------------------------
// The map
// ---------------------------------------------------------------------------

DiffDriveMap::DiffDriveMap(const trajectory::RoundedRectangle &track, std::vector<Box2d> obstacles)
    : track_(track), obstacles_(std::move(obstacles))
{
}

real_t DiffDriveMap::ray_cast(real_t x, real_t y, real_t angle, real_t max_range) const
{
    const real_t dx = std::cos(angle);
    const real_t dy = std::sin(angle);
    real_t nearest = max_range;

    // The slab method: the ray is inside the box between where it enters both slabs and where
    // it leaves either
    for (const auto &b : obstacles_)
    {
        real_t t_in = 0.0;
        real_t t_out = max_range;
        bool misses = false;
        for (const auto &[o, d, lo, hi] : {std::array<real_t, 4>{x, dx, b.x_min, b.x_max},
                                           std::array<real_t, 4>{y, dy, b.y_min, b.y_max}})
        {
            if (std::abs(d) < 1.0e-12)
            {
                if (o < lo || o > hi)
                {
                    misses = true;
                }
                continue;
            }
            real_t t0 = (lo - o) / d;
            real_t t1 = (hi - o) / d;
            if (t0 > t1)
            {
                std::swap(t0, t1);
            }
            t_in = std::max(t_in, t0);
            t_out = std::min(t_out, t1);
        }
        if (!misses && t_in <= t_out)
        {
            nearest = std::min(nearest, t_in);
        }
    }
    return nearest;
}

// ---------------------------------------------------------------------------
// The motion model
// ---------------------------------------------------------------------------

DiffDriveGyroModel::DiffDriveGyroModel()
    : F_(matrix_type::Identity(4, 4)), L_(matrix_type::Zero(4, 3))
{
    state_.set(0, {"x", 0.0});
    state_.set(1, {"y", 0.0});
    state_.set(2, {"yaw", 0.0});
    state_.set(3, {"bias", 0.0});
}

DiffDriveGyroModel::state_type &DiffDriveGyroModel::evaluate(const Input &u)
{
    const real_t yaw = state_[2];
    const real_t bias = state_[3];
    const real_t turn = (u.gyro_z - bias) * u.dt;
    const real_t mid = yaw + 0.5 * turn;
    const real_t c = std::cos(mid);
    const real_t s = std::sin(mid);

    F_.setIdentity();
    F_(0, 2) = -u.distance * s;
    F_(1, 2) = u.distance * c;
    F_(0, 3) = 0.5 * u.dt * u.distance * s;
    F_(1, 3) = -0.5 * u.dt * u.distance * c;
    F_(2, 3) = -u.dt;

    L_.setZero();
    L_(0, 0) = c;
    L_(1, 0) = s;
    L_(0, 1) = -0.5 * u.dt * u.distance * s;
    L_(1, 1) = 0.5 * u.dt * u.distance * c;
    L_(2, 1) = u.dt;
    L_(3, 2) = 1.0;

    state_[0] += u.distance * c;
    state_[1] += u.distance * s;
    state_[2] = wrap_angle(yaw + turn);
    return state_;
}

const DiffDriveGyroModel::matrix_type &DiffDriveGyroModel::get_matrix(const std::string &name) const
{
    if (name == "F")
    {
        return F_;
    }
    if (name == "L")
    {
        return L_;
    }
    throw std::invalid_argument("DiffDriveGyroModel has no matrix " + name);
}

// ---------------------------------------------------------------------------
// The observation models
// ---------------------------------------------------------------------------

namespace
{
using Point = std::array<real_t, 2>;

// A point relative to the robot at state x, in the world
Point to_world(const DynVec<real_t> &x, const Point &p)
{
    const real_t c = std::cos(x[2]);
    const real_t s = std::sin(x[2]);
    return {x[0] + c * p[0] - s * p[1], x[1] + s * p[0] + c * p[1]};
}

// An observation model whose H is the central difference of evaluate with respect to x, y and
// the heading; the bias does not move what the sensors see
template <typename Derived> class NumericalObservation
{
  public:
    DynVec<real_t> evaluate(const DynVec<real_t> &x) const
    {
        const auto &self = static_cast<const Derived &>(*this);
        const DynVec<real_t> z = self.h(x);
        H_ = DynMat<real_t>::Zero(z.size(), x.size());
        const real_t step = 1.0e-6;
        for (int j = 0; j < 3; ++j)
        {
            DynVec<real_t> plus = x;
            DynVec<real_t> minus = x;
            plus[j] += step;
            minus[j] -= step;
            H_.col(j) = ((self.h(plus) - self.h(minus)) / (2.0 * step)).transpose();
        }
        return z;
    }

    const DynMat<real_t> &get_matrix(const std::string &name) const
    {
        if (name != "H")
        {
            throw std::invalid_argument("The observation model has no matrix " + name);
        }
        return H_;
    }

  private:
    mutable DynMat<real_t> H_;
};

// Each point should be on the track: its signed distance to it is measured as 0
class LineObservation : public NumericalObservation<LineObservation>
{
  public:
    LineObservation(const DiffDriveMap &map, const std::vector<Point> &points)
        : map_(map), points_(points)
    {
    }

    DynVec<real_t> h(const DynVec<real_t> &x) const
    {
        DynVec<real_t> z(points_.size());
        for (std::size_t i = 0; i < points_.size(); ++i)
        {
            const Point w = to_world(x, points_[i]);
            z[i] = map_.track().signed_distance(w[0], w[1]);
        }
        return z;
    }

  private:
    const DiffDriveMap &map_;
    const std::vector<Point> &points_;
};

// The shortest ray across the sensor's beam
real_t fan_range(const DiffDriveMap &map, const DiffDriveEKF::RangeSensor &sensor,
                 const DynVec<real_t> &x)
{
    const Point origin = to_world(x, {sensor.x, sensor.y});
    real_t nearest = sensor.max_range;
    for (uint_t r = 0; r < sensor.n_rays; ++r)
    {
        const real_t offset =
            sensor.n_rays == 1
                ? 0.0
                : sensor.beam_angle * (static_cast<real_t>(r) / (sensor.n_rays - 1) - 0.5);
        nearest = std::min(nearest, map.ray_cast(origin[0], origin[1], x[2] + sensor.yaw + offset,
                                                 sensor.max_range));
    }
    return std::max(nearest, sensor.min_range);
}

class RangeObservation : public NumericalObservation<RangeObservation>
{
  public:
    RangeObservation(const DiffDriveMap &map, const DiffDriveEKF::RangeSensor &sensor)
        : map_(map), sensor_(sensor)
    {
    }

    DynVec<real_t> h(const DynVec<real_t> &x) const
    {
        DynVec<real_t> z(1);
        z[0] = fan_range(map_, sensor_, x);
        return z;
    }

  private:
    const DiffDriveMap &map_;
    const DiffDriveEKF::RangeSensor &sensor_;
};
} // namespace

// ---------------------------------------------------------------------------
// The filter
// ---------------------------------------------------------------------------

DiffDriveEKF::DiffDriveEKF(const DiffDriveMap &map, const Pose &start, const Parameters &parameters,
                           const std::array<real_t, 3> &start_sigma)
    : map_(map), p_(parameters), model_(), ekf_(model_)
{
    auto &s = model_.get_state();
    s[0] = start.x;
    s[1] = start.y;
    s[2] = start.yaw;
    s[3] = 0.0;

    DynMat<real_t> P = DynMat<real_t>::Zero(4, 4);
    P(0, 0) = P(1, 1) = start_sigma[0] * start_sigma[0];
    P(2, 2) = start_sigma[1] * start_sigma[1];
    P(3, 3) = start_sigma[2] * start_sigma[2];
    ekf_.set_matrix("P", P);
}

void DiffDriveEKF::predict(real_t distance, real_t gyro_z, real_t dt)
{
    DynMat<real_t> Q = DynMat<real_t>::Zero(3, 3);
    Q(0, 0) = p_.distance_noise * p_.distance_noise * std::abs(distance) + 1.0e-12;
    Q(1, 1) = p_.gyro_noise * p_.gyro_noise;
    Q(2, 2) = p_.bias_walk * p_.bias_walk * dt;
    ekf_.predict({distance, gyro_z, dt}, Q);
}

EKFUpdateResult DiffDriveEKF::update_line(const std::vector<std::array<real_t, 2>> &points,
                                          real_t sigma)
{
    if (points.empty())
    {
        return {};
    }
    const LineObservation model(map_, points);
    const auto n = static_cast<Eigen::Index>(points.size());
    const DynMat<real_t> R = DynMat<real_t>::Identity(n, n) * sigma * sigma;
    const auto result = ekf_.update(model, DynVec<real_t>::Zero(n), R, p_.gate * n);
    wrap_heading_();
    return result;
}

EKFUpdateResult DiffDriveEKF::update_range(const RangeSensor &sensor, real_t measured)
{
    if (measured >= sensor.max_range - 1.0e-6 ||
        predicted_range(sensor, pose()) >= sensor.max_range - 1.0e-6)
    {
        return {};
    }
    const RangeObservation model(map_, sensor);
    DynVec<real_t> z(1);
    z[0] = measured;
    const DynMat<real_t> R = DynMat<real_t>::Identity(1, 1) * p_.range_noise * p_.range_noise;
    const auto result = ekf_.update(model, z, R, p_.gate);
    wrap_heading_();
    return result;
}

real_t DiffDriveEKF::predicted_range(const RangeSensor &sensor, const Pose &pose) const
{
    DynVec<real_t> x(4);
    x << pose.x, pose.y, pose.yaw, 0.0;
    return fan_range(map_, sensor, x);
}

DiffDriveEKF::Pose DiffDriveEKF::pose() const
{
    const auto &s = model_.get_state();
    return {s[0], s[1], s[2]};
}

void DiffDriveEKF::wrap_heading_()
{
    auto &s = model_.get_state();
    s[2] = wrap_angle(s[2]);
}

} // namespace estimation
} // namespace bitsim

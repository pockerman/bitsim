#include "bitrl/trajectory/time_scaling.h"

#include <algorithm>
#include <stdexcept>

namespace bitrl
{
namespace trajectory
{

TimeScaling::TimeScaling(real_t duration) : T_(duration)
{
    if (duration <= 0.0)
    {
        throw std::invalid_argument("TimeScaling: the duration must be positive");
    }
}

real_t TimeScaling::clamp_(real_t t) const noexcept { return std::clamp(t, 0.0, T_); }

// ---------------------------------------------------------------------------
// Cubic
// ---------------------------------------------------------------------------

real_t CubicTimeScaling::s(real_t t) const
{
    const real_t tau = clamp_(t) / T_;
    return tau * tau * (3.0 - 2.0 * tau);
}

real_t CubicTimeScaling::ds(real_t t) const
{
    if (t < 0.0 || t > T_)
    {
        return 0.0;
    }
    const real_t tau = t / T_;
    return 6.0 * tau * (1.0 - tau) / T_;
}

real_t CubicTimeScaling::dds(real_t t) const
{
    if (t < 0.0 || t > T_)
    {
        return 0.0;
    }
    const real_t tau = t / T_;
    return (6.0 - 12.0 * tau) / (T_ * T_);
}

// ---------------------------------------------------------------------------
// Quintic
// ---------------------------------------------------------------------------

real_t QuinticTimeScaling::s(real_t t) const
{
    const real_t tau = clamp_(t) / T_;
    return tau * tau * tau * (10.0 - 15.0 * tau + 6.0 * tau * tau);
}

real_t QuinticTimeScaling::ds(real_t t) const
{
    if (t < 0.0 || t > T_)
    {
        return 0.0;
    }
    const real_t tau = t / T_;
    return 30.0 * tau * tau * (1.0 - tau) * (1.0 - tau) / T_;
}

real_t QuinticTimeScaling::dds(real_t t) const
{
    if (t < 0.0 || t > T_)
    {
        return 0.0;
    }
    const real_t tau = t / T_;
    return 60.0 * tau * (1.0 - tau) * (1.0 - 2.0 * tau) / (T_ * T_);
}

// ---------------------------------------------------------------------------
// Trapezoidal
// ---------------------------------------------------------------------------

TrapezoidalTimeScaling::TrapezoidalTimeScaling(real_t duration, real_t cruise_speed)
    : TimeScaling(duration), v_(cruise_speed), a_(0.0), ta_(0.0)
{
    if (cruise_speed * duration <= 1.0 || cruise_speed * duration > 2.0 + 1.0e-12)
    {
        throw std::invalid_argument("TrapezoidalTimeScaling: the cruise speed must be in "
                                    "(1/T, 2/T]");
    }
    ta_ = duration - 1.0 / cruise_speed;
    a_ = cruise_speed / ta_;
}

TrapezoidalTimeScaling TrapezoidalTimeScaling::with_accel_share(real_t duration, real_t accel_share)
{
    if (accel_share <= 0.0 || accel_share > 0.5)
    {
        throw std::invalid_argument("TrapezoidalTimeScaling: the share must be in (0, 1/2]");
    }
    // ta = T - 1/v = share T
    return TrapezoidalTimeScaling(duration, 1.0 / ((1.0 - accel_share) * duration));
}

real_t TrapezoidalTimeScaling::s(real_t t) const
{
    t = clamp_(t);
    if (t < ta_)
    {
        return 0.5 * a_ * t * t;
    }
    if (t <= T_ - ta_)
    {
        return v_ * t - 0.5 * v_ * ta_;
    }
    const real_t left = T_ - t;
    return 1.0 - 0.5 * a_ * left * left;
}

real_t TrapezoidalTimeScaling::ds(real_t t) const
{
    if (t < 0.0 || t > T_)
    {
        return 0.0;
    }
    if (t < ta_)
    {
        return a_ * t;
    }
    if (t <= T_ - ta_)
    {
        return v_;
    }
    return a_ * (T_ - t);
}

real_t TrapezoidalTimeScaling::dds(real_t t) const
{
    if (t < 0.0 || t > T_)
    {
        return 0.0;
    }
    if (t < ta_)
    {
        return a_;
    }
    if (t <= T_ - ta_)
    {
        return 0.0;
    }
    return -a_;
}

// ---------------------------------------------------------------------------
// PointToPoint
// ---------------------------------------------------------------------------

PointToPoint::PointToPoint(std::vector<real_t> start, std::vector<real_t> end,
                           std::shared_ptr<const TimeScaling> timing)
    : start_(std::move(start)), delta_(end.size()), timing_(std::move(timing))
{
    if (start_.size() != end.size())
    {
        throw std::invalid_argument("PointToPoint: the start and the end must have the same size");
    }
    if (!timing_)
    {
        throw std::invalid_argument("PointToPoint needs a time scaling");
    }
    for (std::size_t i = 0; i < end.size(); ++i)
    {
        delta_[i] = end[i] - start_[i];
    }
}

std::vector<real_t> PointToPoint::position(real_t t) const
{
    const real_t s = timing_->s(t);
    std::vector<real_t> q(start_);
    for (std::size_t i = 0; i < q.size(); ++i)
    {
        q[i] += s * delta_[i];
    }
    return q;
}

std::vector<real_t> PointToPoint::velocity(real_t t) const
{
    const real_t ds = timing_->ds(t);
    std::vector<real_t> v(delta_);
    for (auto &x : v)
    {
        x *= ds;
    }
    return v;
}

std::vector<real_t> PointToPoint::acceleration(real_t t) const
{
    const real_t dds = timing_->dds(t);
    std::vector<real_t> a(delta_);
    for (auto &x : a)
    {
        x *= dds;
    }
    return a;
}

} // namespace trajectory
} // namespace bitrl

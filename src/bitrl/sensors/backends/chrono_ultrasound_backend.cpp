#include "bitrl/bitrl_config.h"

#ifdef BITRL_CHRONO

#include "bitrl/bitrl_consts.h"
#include "bitrl/sensors/backends/chrono_ultrasound_backend.h"

#include <chrono/collision/ChCollisionModel.h>
#include <chrono/collision/ChCollisionSystem.h>

#ifdef BITRL_LOG
#include <boost/log/trivial.hpp>
#endif

#include <algorithm>
#include <stdexcept>
#include <string>

namespace bitrl
{
namespace sensors::backends
{
namespace
{
// How far past a hit on the sensor's own body the ray starts again [m]
const real_t SELF_HIT_STEP = 1.0e-4;

// The ray gives up after passing through the sensor's own body this often
const uint_t MAX_SELF_HITS = 4;
} // namespace

const std::string CHRONO_UltrasonicBackend::BACKEND_TYPE = "SIMULATION";

CHRONO_UltrasonicBackend::CHRONO_UltrasonicBackend()
    : RangeSensorBackendBase(CHRONO_UltrasonicBackend::BACKEND_TYPE), sys_ptr_(nullptr)
{
}

CHRONO_UltrasonicBackend::CHRONO_UltrasonicBackend(chrono::ChSystem &sys,
                                                   std::shared_ptr<chrono::ChBody> body)
    : RangeSensorBackendBase(CHRONO_UltrasonicBackend::BACKEND_TYPE), sys_ptr_(&sys)
{
    this->set_chrono_body(body);
}

void CHRONO_UltrasonicBackend::set_beam(real_t beam_angle, uint_t n_rays)
{
    if (beam_angle < 0.0 || n_rays == 0)
    {
        throw std::logic_error("An ultrasonic sensor needs beam_angle >= 0 and at least one ray");
    }
    beam_angle_ = beam_angle;
    n_rays_ = n_rays;
}

void CHRONO_UltrasonicBackend::load(const nlohmann::json &j)
{
    RangeSensorBackendBase::load(j);

    for (const auto *key : {"mounted_position", "frame_pos"})
    {
        if (j.contains(key))
        {
            const auto &pos = j.at(key);
            position_ = chrono::ChVector3d(pos.at(0).get<real_t>(), pos.at(1).get<real_t>(),
                                           pos.at(2).get<real_t>());
            break;
        }
    }

    const real_t degrees = j.value("beam_angle", beam_angle_ * 180.0 / consts::maths::PI);
    set_beam(degrees * consts::maths::PI / 180.0, j.value("n_rays", n_rays_));
    yaw_ = j.value("yaw", yaw_ * 180.0 / consts::maths::PI) * consts::maths::PI / 180.0;

#ifdef BITRL_LOG
    BOOST_LOG_TRIVIAL(info) << "Loaded sensor backend: CHRONO_UltrasonicBackend";
#endif
}

real_t CHRONO_UltrasonicBackend::cast_(const chrono::ChCollisionSystem &collision_sys,
                                       const chrono::ChVector3d &start,
                                       const chrono::ChVector3d &dir) const
{
    const chrono::ChVector3d end = start + dir * this->max_distance();
    chrono::ChVector3d from = start;
    for (uint_t i = 0; i <= MAX_SELF_HITS; ++i)
    {
        chrono::ChCollisionSystem::ChRayhitResult hit;
        if (!collision_sys.RayHit(from, end, hit))
        {
            return this->max_distance();
        }

        const bool own_body = hit.hitModel && hit.hitModel->GetContactable() ==
                                                  static_cast<chrono::ChContactable *>(body_.get());
        if (!own_body)
        {
            return (hit.abs_hitPoint - start).Length();
        }

        // The ray starts inside the sensor's own body: go on from where it
        // leaves it
        from = hit.abs_hitPoint + dir * SELF_HIT_STEP;
    }
    return this->max_distance();
}

std::vector<real_t> CHRONO_UltrasonicBackend::read_values()
{
    if (!body_)
    {
        throw std::logic_error("CHRONO_UltrasonicBackend has no body: call set_chrono_body()");
    }

    chrono::ChSystem *system = sys_ptr_ ? sys_ptr_ : body_->GetSystem();
    if (system == nullptr)
    {
        throw std::logic_error("The body of the CHRONO_UltrasonicBackend is in no system");
    }

    auto collision_sys = system->GetCollisionSystem();
    if (collision_sys == nullptr)
    {
#ifdef BITRL_LOG
        BOOST_LOG_TRIVIAL(warning)
            << "Collision system is not enabled for backend. Return max_distance";
#endif
        return {this->max_distance()};
    }

    const auto &frame = body_->GetFrameRefToAbs();
    const chrono::ChVector3d start = frame.TransformPointLocalToParent(position_);

    real_t nearest = this->max_distance();
    for (uint_t r = 0; r < n_rays_; ++r)
    {
        // The rays are spread evenly across the cone, about the body's z axis, centred on yaw
        const real_t angle =
            yaw_ +
            (n_rays_ == 1 ? 0.0 : beam_angle_ * (static_cast<real_t>(r) / (n_rays_ - 1) - 0.5));
        const chrono::ChVector3d local_dir(std::cos(angle), std::sin(angle), 0.0);
        const chrono::ChVector3d dir = frame.TransformDirectionLocalToParent(local_dir);
        nearest = std::min(nearest, cast_(*collision_sys, start, dir.GetNormalized()));
    }

    return {this->clamp_distance(nearest)};
}

} // namespace sensors::backends
} // namespace bitrl

#endif

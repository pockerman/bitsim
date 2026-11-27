#include "bitsim/control/fixed_wing_mission.h"
#include "bitsim/bitsim_consts.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace bitsim::control
{

namespace
{
namespace fw = dynamics::fw;
const real_t PI = consts::maths::PI;

// The unit direction from a to b on the ground
std::array<real_t, 2> direction(const MissionWaypoint &a, const MissionWaypoint &b)
{
    const real_t dn = b.north - a.north;
    const real_t de = b.east - a.east;
    const real_t d = std::hypot(dn, de);
    if (d < 1.0e-9)
    {
        throw std::invalid_argument("FixedWingMission: two points of the route are the same");
    }
    return {dn / d, de / d};
}
} // namespace

std::string to_string(MissionPhase phase)
{
    switch (phase)
    {
    case MissionPhase::TAKEOFF_ROLL:
        return "takeoff roll";
    case MissionPhase::ROTATE:
        return "rotate";
    case MissionPhase::ENROUTE:
        return "enroute";
    case MissionPhase::APPROACH:
        return "approach";
    case MissionPhase::FLARE:
        return "flare";
    case MissionPhase::ROLLOUT:
        return "rollout";
    case MissionPhase::STOPPED:
        return "stopped";
    }
    return "unknown";
}

FixedWingMission::FixedWingMission(FixedWingAutopilot &autopilot, const Runway &runway,
                                   const std::vector<MissionWaypoint> &route,
                                   const FixedWingMissionSettings &settings)
    : autopilot_(autopilot), runway_(runway), s_(settings)
{
    if (route.empty())
    {
        throw std::invalid_argument("FixedWingMission: the route needs at least one waypoint");
    }
    const real_t c = std::cos(runway.heading);
    const real_t s = std::sin(runway.heading);
    const auto on_centre_line = [&](real_t along, real_t altitude) {
        return MissionWaypoint{runway.north + along * c, runway.east + along * s, altitude};
    };

    // The threshold, the route, the final approach fix and the aim point
    points_.push_back(on_centre_line(0.0, 0.0));
    points_.insert(points_.end(), route.begin(), route.end());
    points_.push_back(on_centre_line(s_.aim_distance - s_.final_length,
                                     s_.final_length * std::tan(s_.glide_slope)));
    points_.push_back(on_centre_line(s_.aim_distance, 0.0));
}

std::array<real_t, 2> FixedWingMission::runway_coordinates(real_t north, real_t east) const
{
    const real_t dn = north - runway_.north;
    const real_t de = east - runway_.east;
    const real_t c = std::cos(runway_.heading);
    const real_t s = std::sin(runway_.heading);
    return {dn * c + de * s, -dn * s + de * c};
}

real_t FixedWingMission::wheel_height(const dynamics::FixedWingDynamics &aircraft) const
{
    const auto &x = aircraft.state();
    const RealMat3d R = dynamics::FixedWingDynamics::rotation(x[fw::PHI], x[fw::THETA], x[fw::PSI]);
    real_t lowest = std::numeric_limits<real_t>::max();
    for (const auto &wheel : aircraft.parameters().gear)
    {
        const RealColVec3d r(wheel.position[0], wheel.position[1], wheel.position[2]);
        lowest = std::min(lowest, -(x[fw::PD] + (R * r)[2]));
    }
    return lowest;
}

real_t FixedWingMission::follow_(const MissionWaypoint &a, const MissionWaypoint &b, real_t north,
                                 real_t east)
{
    const auto q = direction(a, b);
    const real_t chi_q = std::atan2(q[1], q[0]);
    cross_track_ = -q[1] * (north - a.north) + q[0] * (east - a.east);
    return chi_q - s_.path_angle * 2.0 / PI * std::atan(s_.path_gain * cross_track_);
}

bool FixedWingMission::past_end_of_leg_(real_t north, real_t east) const
{
    const auto &w = points_[leg_ + 1];
    const auto q_in = direction(points_[leg_], w);
    std::array<real_t, 2> n = q_in;
    if (leg_ + 2 < points_.size())
    {
        const auto q_out = direction(w, points_[leg_ + 2]);
        const real_t sn = q_in[0] + q_out[0];
        const real_t se = q_in[1] + q_out[1];
        const real_t d = std::hypot(sn, se);
        if (d > 1.0e-9)
        {
            n = {sn / d, se / d};
        }
    }
    return (north - w.north) * n[0] + (east - w.east) * n[1] >= 0.0;
}

real_t FixedWingMission::steer_(const dynamics::FixedWingDynamics &aircraft)
{
    const auto &x = aircraft.state();
    const real_t chi_c = follow_(points_.front(), points_.back(), x[fw::PN], x[fw::PE]);
    // A positive rudder turns the nose wheel, and the nose, to the left
    return -s_.steering_gain * std::remainder(chi_c - x[fw::PSI], 2.0 * PI);
}

dynamics::FixedWingControls FixedWingMission::update(const dynamics::FixedWingDynamics &aircraft,
                                                     real_t dt)
{
    const auto &x = aircraft.state();
    const auto air = aircraft.forces(x, {});
    const real_t height = wheel_height(aircraft);
    const bool on_ground = air.ground_load > 0.0;

    dynamics::FixedWingControls u;
    switch (phase_)
    {
    case MissionPhase::TAKEOFF_ROLL:
    case MissionPhase::ROTATE:
    {
        const bool rotate = phase_ == MissionPhase::ROTATE;
        u.throttle = 1.0;
        u.aileron = autopilot_.roll_hold(0.0, x[fw::PHI], x[fw::P]);
        u.elevator = autopilot_.pitch_hold(rotate ? s_.climb_pitch : 0.0, x[fw::THETA], x[fw::Q]);
        u.rudder = on_ground ? steer_(aircraft) : autopilot_.sideslip_hold(air.beta, dt);
        commands_ = {s_.cruise_airspeed, points_[1].altitude, runway_.heading};
        if (!rotate && air.airspeed >= s_.rotate_airspeed)
        {
            phase_ = MissionPhase::ROTATE;
        }
        if (rotate && height > s_.clear_height)
        {
            autopilot_.reset();
            phase_ = MissionPhase::ENROUTE;
        }
        break;
    }
    case MissionPhase::ENROUTE:
    case MissionPhase::APPROACH:
    {
        if (past_end_of_leg_(x[fw::PN], x[fw::PE]) && leg_ + 2 < points_.size())
        {
            ++leg_;
        }
        const bool final_leg = leg_ + 2 == points_.size();
        if (final_leg && phase_ == MissionPhase::ENROUTE)
        {
            phase_ = MissionPhase::APPROACH;
        }
        const auto &a = points_[leg_];
        const auto &b = points_[leg_ + 1];
        commands_.course = follow_(a, b, x[fw::PN], x[fw::PE]);
        if (final_leg)
        {
            // Down the glide slope: the wheels at the aim point's height there
            const auto q = direction(a, b);
            const real_t to_go = (b.north - x[fw::PN]) * q[0] + (b.east - x[fw::PE]) * q[1];
            const real_t gear_offset = aircraft.altitude() - height;
            commands_.altitude = gear_offset + std::max(to_go, 0.0) * std::tan(s_.glide_slope);
            commands_.airspeed = s_.approach_airspeed;
        }
        else
        {
            commands_.altitude = b.altitude;
            commands_.airspeed = s_.cruise_airspeed;
        }
        u = autopilot_.update(aircraft, commands_, dt);
        if (final_leg && height < s_.flare_height)
        {
            flare_pitch_ = x[fw::THETA];
            flare_throttle_ = u.throttle;
            sink_integral_ = 0.0;
            phase_ = MissionPhase::FLARE;
        }
        break;
    }
    case MissionPhase::FLARE:
    {
        // A sink rate that falls with the height, to flare_sink_rate on the ground
        const real_t approach_sink = s_.approach_airspeed * std::tan(s_.glide_slope);
        const real_t sink_c =
            s_.flare_sink_rate +
            (approach_sink - s_.flare_sink_rate) * std::clamp(height / s_.flare_height, 0.0, 1.0);
        const real_t error = -aircraft.climb_rate() - sink_c; // positive: sinking too fast
        sink_integral_ += error * dt;
        const real_t theta_c =
            std::clamp(flare_pitch_ + 0.12 * error + 0.15 * sink_integral_, -0.05, 0.25);
        const real_t chi_c = follow_(points_.front(), points_.back(), x[fw::PN], x[fw::PE]);
        commands_ = {s_.approach_airspeed, 0.0, chi_c};
        const real_t phi_c = autopilot_.course_hold(chi_c, aircraft.course(), dt);
        u.aileron = autopilot_.roll_hold(phi_c, x[fw::PHI], x[fw::P]);
        u.rudder = autopilot_.sideslip_hold(air.beta, dt);
        u.elevator = autopilot_.pitch_hold(theta_c, x[fw::THETA], x[fw::Q]);
        // The throttle closes as the wheels near the ground, so the airspeed, and with it the
        // elevator's authority, does not run out before the touchdown
        u.throttle = flare_throttle_ * std::clamp(height / s_.flare_height, 0.0, 1.0);
        if (on_ground)
        {
            const auto where = runway_coordinates(x[fw::PN], x[fw::PE]);
            touchdown_ = Touchdown{aircraft.time(), where[0], where[1], -aircraft.climb_rate(),
                                   air.airspeed};
            phase_ = MissionPhase::ROLLOUT;
        }
        break;
    }
    case MissionPhase::ROLLOUT:
        u.aileron = autopilot_.roll_hold(0.0, x[fw::PHI], x[fw::P]);
        u.elevator = autopilot_.pitch_hold(0.0, x[fw::THETA], x[fw::Q]);
        u.rudder = steer_(aircraft);
        u.brake = air.airspeed < s_.brake_airspeed ? 0.8 : 0.0;
        if (aircraft.ground_speed() < 0.2)
        {
            phase_ = MissionPhase::STOPPED;
        }
        break;
    case MissionPhase::STOPPED:
        u.brake = 1.0;
        break;
    }
    return u;
}

} // namespace bitsim::control

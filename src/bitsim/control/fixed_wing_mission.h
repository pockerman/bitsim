#ifndef BITSIM_CONTROL_FIXED_WING_MISSION_H
#define BITSIM_CONTROL_FIXED_WING_MISSION_H

#include "bitsim/bitsim_types.h"
#include "bitsim/control/fixed_wing_autopilot.h"
#include "bitsim/dynamics/fixed_wing_dynamics.h"

#include <optional>
#include <string>
#include <vector>

namespace bitsim::control
{

///
/// @brief A runway on the ground: it starts at its threshold and runs along its heading
///
struct Runway
{
    real_t north{0.0};    ///< of the threshold [m]
    real_t east{0.0};     ///< of the threshold [m]
    real_t heading{0.0};  ///< [rad]
    real_t length{400.0}; ///< [m]
    real_t width{20.0};   ///< [m]
};

///
/// @brief A point of the route, in the world
///
struct MissionWaypoint
{
    real_t north{0.0};    ///< [m]
    real_t east{0.0};     ///< [m]
    real_t altitude{0.0}; ///< [m]
};

///
/// @brief How the mission flies
///
struct FixedWingMissionSettings
{
    real_t cruise_airspeed{22.0};   ///< [m/s]
    real_t approach_airspeed{18.0}; ///< [m/s]
    real_t rotate_airspeed{17.0};   ///< lift the nose on the takeoff roll [m/s]
    real_t climb_pitch{0.17};       ///< after rotating, until clear of the ground [rad]
    real_t clear_height{8.0};       ///< the height at which the climb out starts [m]
    real_t glide_slope{0.07};       ///< the angle of the approach [rad]
    real_t final_length{550.0};     ///< from the final approach fix to the aim point [m]
    real_t aim_distance{60.0};      ///< of the aim point past the threshold [m]
    real_t flare_height{3.0};       ///< of the wheels, to start the flare [m]
    real_t flare_sink_rate{0.3};    ///< at touchdown [m/s]
    real_t brake_airspeed{12.0};    ///< brake below it on the rollout [m/s]
    real_t path_angle{1.0};         ///< the course towards a path from far away [rad]
    real_t path_gain{0.03};         ///< how sharply to turn onto a path [1/m]
    real_t steering_gain{1.5};      ///< rudder per rad of heading error on the ground
};

enum class MissionPhase
{
    TAKEOFF_ROLL,
    ROTATE,
    ENROUTE,
    APPROACH,
    FLARE,
    ROLLOUT,
    STOPPED
};

std::string to_string(MissionPhase phase);

///
/// @brief Where and how the aircraft touched down
///
struct Touchdown
{
    real_t time{0.0};      ///< [s]
    real_t along{0.0};     ///< past the threshold [m]
    real_t across{0.0};    ///< right of the centre line [m]
    real_t sink_rate{0.0}; ///< [m/s]
    real_t airspeed{0.0};  ///< [m/s]
};

/**
 * @class FixedWingMission
 * @brief Takes off from a runway, flies a route of waypoints and lands back on the runway.
 *
 * - **Takeoff roll:** full throttle, the wings level, the nose wheel steering along the centre
 *   line. At the rotate airspeed it pitches up to climb_pitch.
 * - **Enroute:** from clear_height it follows straight lines from the runway to each waypoint in
 *   turn, then to the final approach fix, final_length before the aim point on the extended centre
 *   line. Each line is followed by commanding a course \f$\chi_q - \chi_\infty \frac{2}{\pi}
 *   \arctan(k e)\f$ with \f$e\f$ the distance right of it, and the next line starts when the
 *   aircraft crosses the half plane that bisects the corner (Beard and McLain, chapters 10, 11).
 * - **Approach:** down the glide slope to the aim point at the approach airspeed.
 * - **Flare:** below flare_height the pitch holds a sink rate that falls with the height, to
 *   flare_sink_rate at touchdown, and the throttle closes with the height.
 * - **Rollout:** on the wheels it steers along the centre line and brakes below
 *   brake_airspeed, until it stops.
 */
class FixedWingMission
{
  public:
    FixedWingMission(FixedWingAutopilot &autopilot, const Runway &runway,
                     const std::vector<MissionWaypoint> &route,
                     const FixedWingMissionSettings &settings = {});

    ///
    /// @brief The controls for the aircraft now, after dt [s]
    ///
    dynamics::FixedWingControls update(const dynamics::FixedWingDynamics &aircraft, real_t dt);

    MissionPhase phase() const noexcept { return phase_; }
    ///
    /// @brief The line being followed: from points()[leg()] to points()[leg() + 1]
    ///
    std::size_t leg() const noexcept { return leg_; }
    const std::vector<MissionWaypoint> &points() const noexcept { return points_; }
    real_t cross_track() const noexcept { return cross_track_; }
    const FixedWingCommands &commands() const noexcept { return commands_; }
    const std::optional<Touchdown> &touchdown() const noexcept { return touchdown_; }

    ///
    /// @return The point on the runway's centre line, along it from the threshold and right of it
    ///
    std::array<real_t, 2> runway_coordinates(real_t north, real_t east) const;

    ///
    /// @return The height of the lowest wheel above the ground [m]
    ///
    real_t wheel_height(const dynamics::FixedWingDynamics &aircraft) const;

  private:
    FixedWingAutopilot &autopilot_;
    Runway runway_;
    FixedWingMissionSettings s_;
    std::vector<MissionWaypoint> points_;
    MissionPhase phase_{MissionPhase::TAKEOFF_ROLL};
    std::size_t leg_{0};
    real_t cross_track_{0.0};
    FixedWingCommands commands_;
    std::optional<Touchdown> touchdown_;
    real_t sink_integral_{0.0};
    real_t flare_pitch_{0.0};
    real_t flare_throttle_{0.0};

    // The course that follows the line from a to b, and the distance right of it
    real_t follow_(const MissionWaypoint &a, const MissionWaypoint &b, real_t north, real_t east);
    // Whether the aircraft has crossed the half plane at the end of the current leg
    bool past_end_of_leg_(real_t north, real_t east) const;
    // The rudder that steers along the centre line on the ground
    real_t steer_(const dynamics::FixedWingDynamics &aircraft);
};

} // namespace bitsim::control

#endif // BITSIM_CONTROL_FIXED_WING_MISSION_H

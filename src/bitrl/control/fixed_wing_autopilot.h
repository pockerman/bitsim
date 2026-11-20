#ifndef BITRL_CONTROL_FIXED_WING_AUTOPILOT_H
#define BITRL_CONTROL_FIXED_WING_AUTOPILOT_H

#include "bitrl/bitrl_types.h"
#include "bitrl/dynamics/fixed_wing_dynamics.h"
#include "bitrl/dynamics/fixed_wing_trim.h"

namespace bitrl::control
{

///
/// @brief What the autopilot holds
///
struct FixedWingCommands
{
    real_t airspeed{25.0};  ///< [m/s]
    real_t altitude{100.0}; ///< [m]
    real_t course{0.0};     ///< the direction over the ground [rad]
};

///
/// @brief The gains of the loops, worked out from the linearised aircraft
///
struct FixedWingAutopilotGains
{
    real_t roll_kp{0.0}, roll_kd{0.0};
    real_t course_kp{0.0}, course_ki{0.0};
    real_t sideslip_kp{0.0}, sideslip_ki{0.0};
    real_t pitch_kp{0.0}, pitch_kd{0.0}, pitch_dc_gain{1.0};
    real_t altitude_kp{0.0}, altitude_ki{0.0};
    real_t throttle_kp{0.0}, throttle_ki{0.0};
    real_t airspeed_pitch_kp{0.0}, airspeed_pitch_ki{0.0};
};

///
/// @brief How the loops are designed, and the limits of what they command
///
struct FixedWingAutopilotDesign
{
    real_t airspeed{25.0};       ///< the trim the gains are designed at [m/s]
    real_t roll_error_max{0.35}; ///< the roll error at which the aileron reaches its limit [rad]
    real_t roll_damping{0.9};
    real_t course_separation{15.0}; ///< how much slower the course loop is than the roll loop
    real_t course_damping{1.0};
    real_t sideslip_error_max{0.17};
    real_t sideslip_damping{0.9};
    real_t pitch_error_max{0.17};
    real_t pitch_damping{0.9};
    real_t altitude_separation{20.0};
    real_t altitude_damping{0.9};
    real_t airspeed_frequency{0.6}; ///< of the throttle loop [rad/s]
    real_t airspeed_damping{1.0};
    real_t airspeed_pitch_separation{10.0};
    real_t airspeed_pitch_damping{1.0};

    real_t max_bank{0.52};      ///< [rad]
    real_t max_pitch{0.35};     ///< [rad]
    real_t min_pitch{-0.26};    ///< [rad]
    real_t altitude_band{15.0}; ///< hold the altitude within it; climb or descend outside [m]
};

/**
 * @class FixedWingAutopilot
 * @brief Holds airspeed, altitude and course with loops closed one inside the other, as in
 * Beard and McLain, chapter 6.
 *
 * - **Lateral:** the course loop sets a bank, the roll loop the aileron; the rudder holds the
 *   sideslip at 0, so the turns are coordinated.
 * - **Longitudinal:** within altitude_band of the commanded altitude, the altitude loop sets a
 *   pitch, the pitch loop the elevator, and the throttle holds the airspeed. Further below, it
 *   climbs at full throttle and the pitch holds the airspeed; further above, it descends at idle.
 *
 * The gains come from the linearised aircraft at a trim: each loop is designed for a damping and
 * a natural frequency, and each outer loop is a number of times slower than the loop inside it.
 * The integrators stop while what they drive is at its limit.
 */
class FixedWingAutopilot
{
  public:
    FixedWingAutopilot(const dynamics::FixedWingParameters &parameters,
                       const FixedWingAutopilotDesign &design = {});

    ///
    /// @brief The controls for the commands, after dt [s]
    ///
    dynamics::FixedWingControls update(const dynamics::FixedWingDynamics &aircraft,
                                       const FixedWingCommands &commands, real_t dt);

    // The loops, for a mission that drives some of them itself
    real_t roll_hold(real_t phi_c, real_t phi, real_t p) const;
    real_t course_hold(real_t chi_c, real_t chi, real_t dt);
    real_t sideslip_hold(real_t beta, real_t dt);
    real_t pitch_hold(real_t theta_c, real_t theta, real_t q) const;
    real_t altitude_hold(real_t h_c, real_t h, real_t dt);
    real_t airspeed_with_throttle(real_t Va_c, real_t Va, real_t dt);
    real_t airspeed_with_pitch(real_t Va_c, real_t Va, real_t dt);

    ///
    /// @brief Forget the integrators
    ///
    void reset();

    const FixedWingAutopilotGains &gains() const noexcept { return gains_; }
    const FixedWingAutopilotDesign &design() const noexcept { return design_; }
    const dynamics::FixedWingTrim &trim() const noexcept { return trim_; }

    enum class AltitudeMode
    {
        HOLD,
        CLIMB,
        DESCEND
    };
    AltitudeMode altitude_mode() const noexcept { return altitude_mode_; }

  private:
    dynamics::FixedWingParameters p_;
    FixedWingAutopilotDesign design_;
    dynamics::FixedWingTrim trim_;
    FixedWingAutopilotGains gains_;
    AltitudeMode altitude_mode_{AltitudeMode::HOLD};

    real_t course_integral_{0.0};
    real_t sideslip_integral_{0.0};
    real_t altitude_integral_{0.0};
    real_t throttle_integral_{0.0};
    real_t airspeed_pitch_integral_{0.0};
};

} // namespace bitrl::control

#endif // BITRL_CONTROL_FIXED_WING_AUTOPILOT_H

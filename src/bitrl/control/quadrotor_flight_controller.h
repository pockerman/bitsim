#ifndef BITRL_CONTROL_QUADROTOR_FLIGHT_CONTROLLER_H
#define BITRL_CONTROL_QUADROTOR_FLIGHT_CONTROLLER_H

#include "bitrl/bitrl_types.h"
#include "bitrl/control/rc_command.h"
#include "bitrl/dynamics/quadrotor_dynamics.h"

namespace bitrl
{
namespace control
{

///
/// @ingroup bitrl_control
/// @brief The rotor speeds [rad/s] of a plus quadrotor (see dynamics::QuadrotorDynamics) that give
/// the thrust T [N] and the torques tau [N m] about the body's x, y and z axes.
///
/// The thrust and torques are linear in the squares of the speeds, \f$s_i = \omega_i^2\f$:
///
/// \f[
/// s_{1,3} = \frac{T}{4k_1} \pm \frac{\tau_\theta}{2lk_1} + \frac{\tau_\psi}{4k_2}, \qquad
/// s_{2,4} = \frac{T}{4k_1} \mp \frac{\tau_\phi}{2lk_1} - \frac{\tau_\psi}{4k_2}
/// \f]
///
/// Each \f$s_i\f$ is clamped to \f$[0, \omega_{max}^2]\f$: a rotor cannot push down, nor spin
/// faster than its motor allows, and then the torques are not all met.
///
RealVec quadrotor_mixer(const dynamics::QuadrotorDynamicsConfig &config, real_t thrust,
                        const RealColVec3d &tau, real_t max_rotor_speed);

/**
 * @class QuadrotorFlightController
 * @ingroup bitrl_control
 * @brief The flight controller of a toy quadrotor in angle mode: it turns the sticks of an
 * RCCommand into rotor speeds.
 *
 * - The throttle sets the thrust directly, as a share of the most the rotors can give; there is
 *   no height hold, so the pilot holds the height. At hover_throttle() the thrust carries the
 *   weight.
 * - The roll and pitch sticks set the angles to tilt to, up to max_tilt; letting go levels the
 *   quadrotor. A PD controller on each angle, with the body rates for the D part, gives the
 *   torque \f$\tau = J (\omega_n^2 (\phi_{sp} - \phi) - 2 \zeta \omega_n p)\f$.
 * - The yaw stick sets the rate of turn, up to max_yaw_rate; a P controller on the rate gives
 *   the yaw torque.
 * - quadrotor_mixer() turns the thrust and the torques into rotor speeds. Disarmed, the rotors
 *   stop.
 *
 * It reads the attitude and the rates of the quadrotor directly; a real flight controller would
 * estimate them from its IMU.
 */
class QuadrotorFlightController
{
  public:
    struct Parameters
    {
        real_t max_tilt{0.436};          ///< [rad] at full roll or pitch stick, 25 degrees
        real_t max_yaw_rate{2.09};       ///< [rad/s] at full yaw stick, 120 deg/s
        real_t thrust_to_weight{2.0};    ///< the most the rotors can lift, in weights
        real_t attitude_frequency{12.0}; ///< \f$\omega_n\f$ of the angle loops [rad/s]
        real_t attitude_damping{0.8};    ///< \f$\zeta\f$ of the angle loops
        real_t yaw_rate_gain{8.0};       ///< of the yaw rate loop [1/s]
    };

    explicit QuadrotorFlightController(const dynamics::QuadrotorDynamicsConfig &config);
    QuadrotorFlightController(const dynamics::QuadrotorDynamicsConfig &config,
                              const Parameters &parameters);

    ///
    /// @brief The rotor speeds [rad/s] for the command and the state of the quadrotor
    ///
    RealVec rotor_speeds(const RCCommand &command,
                         const dynamics::QuadrotorDynamics &quadrotor) const;

    ///
    /// @brief The throttle whose thrust carries the weight: 1 / thrust_to_weight
    ///
    real_t hover_throttle() const noexcept { return 1.0 / p_.thrust_to_weight; }

    ///
    /// @brief The fastest the rotors spin [rad/s], at full throttle
    ///
    real_t max_rotor_speed() const noexcept { return max_rotor_speed_; }

    const Parameters &parameters() const noexcept { return p_; }

  private:
    dynamics::QuadrotorDynamicsConfig config_;
    Parameters p_;
    real_t max_thrust_;
    real_t max_rotor_speed_;
};

} // namespace control
} // namespace bitrl

#endif // BITRL_CONTROL_QUADROTOR_FLIGHT_CONTROLLER_H

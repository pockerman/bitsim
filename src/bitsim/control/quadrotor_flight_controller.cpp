#include "bitsim/control/quadrotor_flight_controller.h"
#include "bitsim/bitsim_consts.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace bitsim
{
namespace control
{

RealVec quadrotor_mixer(const dynamics::QuadrotorDynamicsConfig &config, real_t thrust,
                        const RealColVec3d &tau, real_t max_rotor_speed)
{
    const real_t k1 = config.k_1;
    const real_t k2 = config.k_2;
    const real_t l = config.l;

    const real_t base = thrust / (4.0 * k1);
    const real_t roll = tau[0] / (2.0 * l * k1);
    const real_t pitch = tau[1] / (2.0 * l * k1);
    const real_t yaw = tau[2] / (4.0 * k2);

    const real_t s_max = max_rotor_speed * max_rotor_speed;
    const auto speed = [s_max](real_t s) { return std::sqrt(std::clamp(s, 0.0, s_max)); };

    RealVec w(4);
    w[0] = speed(base + pitch + yaw); // front
    w[1] = speed(base - roll - yaw);  // right
    w[2] = speed(base - pitch + yaw); // back
    w[3] = speed(base + roll - yaw);  // left
    return w;
}

QuadrotorFlightController::QuadrotorFlightController(
    const dynamics::QuadrotorDynamicsConfig &config)
    : QuadrotorFlightController(config, Parameters{})
{
}

QuadrotorFlightController::QuadrotorFlightController(
    const dynamics::QuadrotorDynamicsConfig &config, const Parameters &parameters)
    : config_(config), p_(parameters), max_thrust_(0.0), max_rotor_speed_(0.0)
{
    if (p_.thrust_to_weight <= 1.0)
    {
        throw std::invalid_argument(
            "QuadrotorFlightController: the rotors must be able to lift more than the weight");
    }
    max_thrust_ = p_.thrust_to_weight * config_.mass * consts::maths::G;
    max_rotor_speed_ = std::sqrt(max_thrust_ / (4.0 * config_.k_1));
}

RealVec QuadrotorFlightController::rotor_speeds(const RCCommand &input,
                                                const dynamics::QuadrotorDynamics &quadrotor) const
{
    const RCCommand command = input.clamped();
    if (!command.armed)
    {
        return RealVec::Zero(4);
    }

    // The sticks: roll right is a positive roll, pitch forward is nose down, a negative pitch in
    // the NED body frame, and yaw right is clockwise from above, a positive yaw rate
    const real_t roll_sp = command.roll * p_.max_tilt;
    const real_t pitch_sp = -command.pitch * p_.max_tilt;
    const real_t yaw_rate_sp = command.yaw * p_.max_yaw_rate;

    const RealColVec3d rates = quadrotor.get_angular_velocity();
    const real_t wn = p_.attitude_frequency;
    const real_t kp = wn * wn;
    const real_t kd = 2.0 * p_.attitude_damping * wn;

    RealColVec3d tau;
    tau[0] = config_.Jx * (kp * (roll_sp - quadrotor.phi()) - kd * rates[0]);
    tau[1] = config_.Jy * (kp * (pitch_sp - quadrotor.theta()) - kd * rates[1]);
    tau[2] = config_.Jz * p_.yaw_rate_gain * (yaw_rate_sp - rates[2]);

    return quadrotor_mixer(config_, command.throttle * max_thrust_, tau, max_rotor_speed_);
}

} // namespace control
} // namespace bitsim

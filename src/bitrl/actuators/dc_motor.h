#ifndef DC_MOTOR_H
#define DC_MOTOR_H

#include "bitrl/actuators/backends/dc_motor_backend_base.h"
#include "bitrl/actuators/backends/sim_dc_motor_backend.h"
#include "bitrl/bitrl_consts.h"
#include "bitrl/bitrl_types.h"

#include <limits>
#include <memory>
#include <string>

namespace bitrl
{
namespace actuators
{

/**
 * @class DCMotor
 * @ingroup bitrl_actuators
 * @brief A DC motor with an encoder, run in speed control. The application sets
 * the target speed of the output shaft and reads back its angle and speed.
 * The backend decides what the motor is:
 *
 * - backends::SIM_DCMotorBackend, a model of the motor that runs in the
 *   simulation only
 * - backends::MQTT_DCMotorBackend, a real motor on a board that talks MQTT
 *
 * Call step() once per simulation step.
 */
class DCMotor
{
  public:
    using OutputType = backends::DCMotorBackendBase::OutputType;

    ///
    /// @brief A motor with a SIM_DCMotorBackend
    ///
    static std::shared_ptr<DCMotor>
    create_simulated(const backends::DCMotorParameters &parameters = backends::DCMotorParameters(),
                     const std::string &name = consts::INVALID_STR);

    explicit DCMotor(std::shared_ptr<backends::DCMotorBackendBase> backend,
                     const std::string &name = consts::INVALID_STR);

    ///
    /// @brief Set the target speed of the output shaft [rad/s]. It is clamped
    /// to the maximum speed
    ///
    void set_speed(real_t speed);

    ///
    /// @return The target speed of the output shaft [rad/s]
    ///
    real_t target_speed() const noexcept { return target_speed_; }

    ///
    /// @brief Limit the target speed to [-max_speed, max_speed]
    ///
    void set_max_speed(real_t max_speed);
    real_t max_speed() const noexcept { return max_speed_; }

    ///
    /// @brief Advance the motor by dt. A model moves its own shaft
    ///
    void step(real_t dt) { backend_->step(dt); }

    ///
    /// @brief Advance the motor by dt, with the shaft moved by a physics engine
    ///
    void step(real_t dt, const ShaftState &shaft) { backend_->step(dt, shaft); }

    ///
    /// @return The angle of the output shaft [rad]
    ///
    real_t angle() const { return backend_->angle(); }

    ///
    /// @return The speed of the output shaft [rad/s]
    ///
    real_t speed() const { return backend_->speed(); }

    ///
    /// @return The angle of the output shaft as its encoder measures it [rad].
    /// See backends::DCMotorBackendBase::encoder_angle
    ///
    real_t encoder_angle() const { return backend_->encoder_angle(); }

    ///
    /// @return The torque the motor puts on the output shaft [N m], if the
    /// backend knows it
    ///
    real_t torque() const { return backend_->torque(); }

    ///
    /// @return How the motor couples to a physics engine
    ///
    OutputType output_type() const noexcept { return backend_->output_type(); }

    bool is_ready() const { return backend_->is_ready(); }

    const std::string &name() const noexcept { return name_; }
    const std::string &backend_type_str() const noexcept { return backend_->backend_type_str(); }
    std::shared_ptr<backends::DCMotorBackendBase> backend_ptr() const noexcept { return backend_; }

  private:
    std::shared_ptr<backends::DCMotorBackendBase> backend_;
    std::string name_;
    real_t target_speed_{0.0};
    real_t max_speed_{std::numeric_limits<real_t>::max()};
};

} // namespace actuators
} // namespace bitrl

#endif // DC_MOTOR_H

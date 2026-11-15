#ifndef SIM_DC_MOTOR_BACKEND_H
#define SIM_DC_MOTOR_BACKEND_H

#include "bitrl/actuators/backends/dc_motor_backend_base.h"
#include "bitrl/bitrl_types.h"
#include "bitrl/extern/nlohmann/json/json.hpp"

#include <string>

namespace bitrl
{
namespace actuators::backends
{

///
/// @brief The constants of a brushed DC gear motor with an encoder, and of the
/// speed controller that runs it. The defaults are those of a 12 V gear motor
/// with a 30:1 gearbox, about 300 rpm at the output, such as a JGA25-370
///
struct DCMotorParameters
{
    real_t resistance{7.0};       ///< armature resistance [Ohm]
    real_t inductance{2.0e-3};    ///< armature inductance [H]
    real_t emf_constant{0.0127};  ///< back-EMF and torque constant, at the rotor [V s/rad]
    real_t rotor_inertia{1.0e-6}; ///< [kg m^2]
    real_t friction{1.0e-6};      ///< viscous friction at the rotor [N m s/rad]
    real_t gear_ratio{30.0};      ///< rotor turns per output turn
    real_t supply_voltage{12.0};  ///< [V]

    // The speed controller, as the board runs it
    real_t speed_kp{0.03};       ///< duty cycle per rad/s of error
    real_t speed_ki{0.1};        ///< duty cycle per rad of error
    real_t speed_filter{0.02};   ///< time constant of the speed filter [s]
    real_t control_period{0.01}; ///< [s]
    uint_t encoder_counts{660};  ///< counts per output turn

    ///
    /// @brief Read the parameters that are in the JSON object. The others keep
    /// their defaults
    ///
    void read(const nlohmann::json &j);

    ///
    /// @return The output speed with the full supply voltage and no load [rad/s]
    ///
    real_t no_load_speed() const { return supply_voltage / (gear_ratio * emf_constant); }

    ///
    /// @return The output torque with the full supply voltage at stall [N m]
    ///
    real_t stall_torque() const { return gear_ratio * emf_constant * supply_voltage / resistance; }
};

/**
 * @class SIM_DCMotorBackend
 * @ingroup bitrl_actuators
 * @brief A model of a DC gear motor, its encoder and its speed controller.
 *
 * The electrical part is
 * \f$ L \, di/dt = V - R i - K_e \omega_r \f$, with the rotor speed
 * \f$ \omega_r = N \omega \f$. The motor puts the torque \f$ N K_e i \f$ on the
 * output shaft. The controller runs every control_period: it measures the speed
 * from the encoder counts, filters it, and sets the duty cycle of the voltage
 * with a PI controller and a back-EMF feedforward, as a board would.
 *
 * step(dt) moves the rotor with its own inertia and an optional load torque.
 * step(dt, shaft) takes the shaft from a physics engine instead, which then
 * applies torque() to it.
 */
class SIM_DCMotorBackend final : public DCMotorBackendBase
{
  public:
    static const std::string BACKEND_TYPE;

    explicit SIM_DCMotorBackend(const DCMotorParameters &parameters = DCMotorParameters());

    OutputType output_type() const noexcept override { return OutputType::TORQUE; }

    void set_target_speed(real_t speed) override { target_speed_ = speed; }

    void step(real_t dt) override;

    void step(real_t dt, const ShaftState &shaft) override;

    real_t angle() const override { return angle_; }

    real_t speed() const override { return speed_; }

    real_t torque() const override;

    ///
    /// @brief The load on the output shaft when the motor moves its own rotor
    /// [N m]. It opposes positive rotation
    ///
    void set_load_torque(real_t torque) noexcept { load_torque_ = torque; }

    ///
    /// @return The angle as the encoder measures it [rad]
    ///
    real_t encoder_angle() const noexcept;

    ///
    /// @return The speed as the controller measures it [rad/s]
    ///
    real_t measured_speed() const noexcept { return measured_speed_; }

    real_t current() const noexcept { return current_; }
    real_t voltage() const noexcept { return voltage_; }
    real_t target_speed() const noexcept { return target_speed_; }
    const DCMotorParameters &parameters() const noexcept { return parameters_; }

  private:
    void control_(real_t dt);
    void advance_(real_t dt, bool own_rotor);

    DCMotorParameters parameters_;

    real_t target_speed_{0.0};
    real_t load_torque_{0.0};

    // The motor
    real_t current_{0.0};
    real_t angle_{0.0};
    real_t speed_{0.0};

    // The controller
    real_t voltage_{0.0};
    real_t integral_{0.0};
    real_t measured_speed_{0.0};
    long last_counts_{0};
    real_t since_control_{0.0};
};

} // namespace actuators::backends
} // namespace bitrl

#endif // SIM_DC_MOTOR_BACKEND_H

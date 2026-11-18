#include "bitrl/actuators/backends/sim_dc_motor_backend.h"
#include "bitrl/bitrl_consts.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace bitrl
{
namespace actuators::backends
{
namespace
{
// The electrical time constant of a small motor is a fraction of a
// millisecond, so the model takes steps much smaller than that
const real_t MODEL_STEP = 2.0e-5;

void read_if_present(const nlohmann::json &j, const std::string &key, real_t &value)
{
    if (j.contains(key))
    {
        value = j.at(key).get<real_t>();
    }
}

long encoder_counts(real_t angle, uint_t counts_per_turn)
{
    return static_cast<long>(std::floor(angle * counts_per_turn / (2.0 * consts::maths::PI)));
}
} // namespace

const std::string SIM_DCMotorBackend::BACKEND_TYPE = "SIMULATION";

void DCMotorParameters::read(const nlohmann::json &j)
{
    read_if_present(j, "resistance", resistance);
    read_if_present(j, "inductance", inductance);
    read_if_present(j, "emf_constant", emf_constant);
    read_if_present(j, "rotor_inertia", rotor_inertia);
    read_if_present(j, "friction", friction);
    read_if_present(j, "gear_ratio", gear_ratio);
    read_if_present(j, "supply_voltage", supply_voltage);
    read_if_present(j, "speed_kp", speed_kp);
    read_if_present(j, "speed_ki", speed_ki);
    read_if_present(j, "speed_filter", speed_filter);
    read_if_present(j, "control_period", control_period);
    if (j.contains("encoder_counts"))
    {
        encoder_counts = j.at("encoder_counts").get<uint_t>();
    }
}

SIM_DCMotorBackend::SIM_DCMotorBackend(const DCMotorParameters &parameters)
    : DCMotorBackendBase(BACKEND_TYPE), parameters_(parameters)
{
    if (parameters_.resistance <= 0.0 || parameters_.inductance <= 0.0 ||
        parameters_.emf_constant <= 0.0 || parameters_.rotor_inertia <= 0.0 ||
        parameters_.gear_ratio <= 0.0 || parameters_.supply_voltage <= 0.0 ||
        parameters_.control_period <= 0.0 || parameters_.encoder_counts == 0)
    {
        throw std::logic_error("SIM_DCMotorBackend: the motor parameters must be positive");
    }
}

real_t SIM_DCMotorBackend::torque() const
{
    const auto &p = parameters_;
    return p.gear_ratio * (p.emf_constant * current_ - p.friction * p.gear_ratio * speed_);
}

real_t SIM_DCMotorBackend::encoder_angle() const
{
    return encoder_counts(angle_, parameters_.encoder_counts) * 2.0 * consts::maths::PI /
           parameters_.encoder_counts;
}

void SIM_DCMotorBackend::step(real_t dt)
{
    control_(dt);
    advance_(dt, true);
}

void SIM_DCMotorBackend::step(real_t dt, const ShaftState &shaft)
{
    angle_ = shaft.angle;
    speed_ = shaft.speed;
    control_(dt);
    advance_(dt, false);
}

void SIM_DCMotorBackend::control_(real_t dt)
{
    const auto &p = parameters_;
    since_control_ += dt;
    if (since_control_ < p.control_period - 1.0e-9)
    {
        return;
    }
    const real_t period = since_control_;
    since_control_ = 0.0;

    // The speed from the change of the encoder counts, filtered
    const long counts = encoder_counts(angle_, p.encoder_counts);
    const real_t raw_speed =
        (counts - last_counts_) * 2.0 * consts::maths::PI / (p.encoder_counts * period);
    last_counts_ = counts;
    measured_speed_ += (raw_speed - measured_speed_) * period / (p.speed_filter + period);

    // The feedforward is the duty cycle that balances the back-EMF at the
    // target speed. The PI controller corrects the rest
    const real_t feedforward = target_speed_ * p.gear_ratio * p.emf_constant / p.supply_voltage;
    const real_t error = target_speed_ - measured_speed_;
    const real_t unclamped = feedforward + p.speed_kp * error + integral_;
    const real_t duty = std::clamp(unclamped, -1.0, 1.0);

    // Anti-windup: stop integrating while the output is saturated and the
    // error would push it further
    if (duty == unclamped || error * unclamped < 0.0)
    {
        integral_ += p.speed_ki * error * period;
    }
    voltage_ = duty * p.supply_voltage;
}

void SIM_DCMotorBackend::advance_(real_t dt, bool own_rotor)
{
    const auto &p = parameters_;
    const int n = std::max(1, static_cast<int>(std::ceil(dt / MODEL_STEP)));
    const real_t h = dt / n;

    for (int k = 0; k < n; ++k)
    {
        const real_t rotor_speed = p.gear_ratio * speed_;
        current_ +=
            h * (voltage_ - p.resistance * current_ - p.emf_constant * rotor_speed) / p.inductance;

        if (own_rotor)
        {
            const real_t rotor_torque =
                p.emf_constant * current_ - p.friction * rotor_speed - load_torque_ / p.gear_ratio;
            speed_ += h * rotor_torque / (p.rotor_inertia * p.gear_ratio);
            angle_ += h * speed_;
        }
    }
}

} // namespace actuators::backends
} // namespace bitrl

#include "bitrl/actuators/bldc_motor.h"
#include "bitrl/bitrl_consts.h"

#include <cmath>

namespace bitrl
{
namespace actuators
{

namespace
{
const real_t PI = consts::maths::PI;
const real_t DEG = PI / 180.0;
} // namespace

DCEquivalentState dc_equivalent_step(const BLDCMotorParameters &m, const DCEquivalentState &s,
                                     real_t V, real_t T_load, real_t dt)
{
    const real_t di = (V - m.R * s.i - m.Ke * s.w) / m.L;
    const real_t dw = (m.Kt() * s.i - m.b * s.w - T_load) / m.J;
    return {s.i + dt * di, s.w + dt * dw};
}

BLDCMotor::BLDCMotor(const BLDCMotorParameters &m, real_t hall_offset)
    : m_(m), hall_offset_(hall_offset),
      // The average of the line-to-line back-EMF over a sector is 3/pi of its peak. This makes
      // the average equal to that of the DC-equivalent model
      k_line_(PI / 3.0 * m.Ke), k_phase_(k_line_ / std::sqrt(3.0))
{
}

int BLDCMotor::hall_code(real_t theta_e)
{
    const bool h1 = std::sin(theta_e - 30.0 * DEG) > 0.0;
    const bool h2 = std::sin(theta_e - 150.0 * DEG) > 0.0;
    const bool h3 = std::sin(theta_e + 90.0 * DEG) > 0.0;
    return h1 + 2 * h2 + 4 * h3;
}

PhasePair BLDCMotor::commutation(int hall_code)
{
    switch (hall_code)
    {
    case 4:
        return {2, 1}; // c to b, rotor at -30 to 30 electrical degrees
    case 5:
        return {0, 1}; // a to b, 30 to 90
    case 1:
        return {0, 2}; // a to c, 90 to 150
    case 3:
        return {1, 2}; // b to c, 150 to 210
    case 2:
        return {1, 0}; // b to a, 210 to 270
    case 6:
        return {2, 0}; // c to a, 270 to 330
    default:
        return {0, 0};
    }
}

real_t BLDCMotor::phase_emf(int phase, real_t theta_e) const
{
    return k_phase_ * std::sin(theta_e - phase * 120.0 * DEG);
}

real_t BLDCMotor::sensed_angle(real_t theta) const { return m_.pole_pairs * theta + hall_offset_; }

real_t BLDCMotor::step(real_t V, real_t theta, real_t w, real_t dt)
{
    const real_t theta_e = m_.pole_pairs * theta;
    pair_ = commutation(hall_code(sensed_angle(theta)));

    // The two active phases are in series: R and L between two terminals, and the difference
    // of their back-EMFs
    const real_t k = phase_emf(pair_.high, theta_e) - phase_emf(pair_.low, theta_e);
    i_ += dt * (V - m_.R * i_ - k * w) / m_.L;

    // The electrical power k w i becomes mechanical power T w
    return k * i_;
}

void BLDCRotor::step(const BLDCMotorParameters &m, real_t T_motor, real_t T_load, real_t dt)
{
    w += dt * (T_motor - m.b * w - T_load) / m.J;
    theta += dt * w;
}

} // namespace actuators
} // namespace bitrl

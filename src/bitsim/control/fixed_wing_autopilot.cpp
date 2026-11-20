#include "bitsim/control/fixed_wing_autopilot.h"
#include "bitsim/bitsim_consts.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace bitsim::control
{

namespace
{
namespace fw = dynamics::fw;
const real_t G = consts::maths::G;

real_t sign(real_t x) { return x < 0.0 ? -1.0 : 1.0; }

// A PI term whose integral stops growing while the output is at a limit
real_t limited_pi(real_t error, real_t kp, real_t ki, real_t offset, real_t lo, real_t hi,
                  real_t &integral, real_t dt)
{
    const real_t next = integral + error * dt;
    const real_t unlimited = offset + kp * error + ki * next;
    const real_t out = std::clamp(unlimited, lo, hi);
    if (out == unlimited)
    {
        integral = next;
    }
    return out;
}
} // namespace

FixedWingAutopilot::FixedWingAutopilot(const dynamics::FixedWingParameters &parameters,
                                       const FixedWingAutopilotDesign &design)
    : p_(parameters), design_(design)
{
    trim_ = dynamics::trim_level_flight(p_, design_.airspeed);
    if (!trim_.converged)
    {
        throw std::invalid_argument("FixedWingAutopilot: the aircraft has no level trim at the "
                                    "design airspeed");
    }
    const auto lin = dynamics::linearise(p_, trim_);
    const auto &A = lin.A;
    const auto &B = lin.B;
    const real_t Va = design_.airspeed;
    auto &g = gains_;

    // Roll: p' = -a1 p + a2 aileron
    const real_t a_phi1 = -A(fw::P, fw::P);
    const real_t a_phi2 = B(fw::P, 1);
    g.roll_kp = p_.max_aileron / design_.roll_error_max * sign(a_phi2);
    const real_t w_phi = std::sqrt(std::abs(a_phi2 * g.roll_kp));
    // The aircraft's own roll damping may be enough: then no derivative term
    g.roll_kd = (2.0 * design_.roll_damping * w_phi - a_phi1) / a_phi2;
    if (g.roll_kd * a_phi2 < 0.0)
    {
        g.roll_kd = 0.0;
    }

    // Course: chi' = g / Va tan(phi), with the roll loop much faster
    const real_t w_chi = w_phi / design_.course_separation;
    g.course_kp = 2.0 * design_.course_damping * w_chi * Va / G;
    g.course_ki = w_chi * w_chi * Va / G;

    // Sideslip: beta' = -a1 beta + a2 rudder
    const real_t a_beta1 = -A(fw::V, fw::V);
    const real_t a_beta2 = B(fw::V, 2) / Va;
    g.sideslip_kp = p_.max_rudder / design_.sideslip_error_max * sign(a_beta2);
    const real_t w_beta = (a_beta1 + a_beta2 * g.sideslip_kp) / (2.0 * design_.sideslip_damping);
    g.sideslip_ki = w_beta * w_beta / a_beta2;

    // Pitch: theta'' = -a1 theta' - a2 theta + a3 elevator
    const real_t a_th1 = -A(fw::Q, fw::Q);
    const real_t a_th2 = -A(fw::Q, fw::W) * Va;
    const real_t a_th3 = B(fw::Q, 0);
    g.pitch_kp = p_.max_elevator / design_.pitch_error_max * sign(a_th3);
    const real_t w_th = std::sqrt(a_th2 + g.pitch_kp * a_th3);
    g.pitch_kd = (2.0 * design_.pitch_damping * w_th - a_th1) / a_th3;
    g.pitch_dc_gain = g.pitch_kp * a_th3 / (a_th2 + g.pitch_kp * a_th3);

    // Altitude: h' = Va theta, through the pitch loop's gain
    const real_t w_h = w_th / design_.altitude_separation;
    g.altitude_kp = 2.0 * design_.altitude_damping * w_h / (g.pitch_dc_gain * Va);
    g.altitude_ki = w_h * w_h / (g.pitch_dc_gain * Va);

    // Airspeed with the throttle: Va' = -a1 Va + a2 throttle
    const real_t a_V1 = -A(fw::U, fw::U);
    const real_t a_V2 = B(fw::U, 3);
    const real_t w_V = design_.airspeed_frequency;
    g.throttle_kp = (2.0 * design_.airspeed_damping * w_V - a_V1) / a_V2;
    g.throttle_ki = w_V * w_V / a_V2;

    // Airspeed with the pitch: Va' = -a1 Va - g theta
    const real_t w_V2 = w_th / design_.airspeed_pitch_separation;
    g.airspeed_pitch_kp =
        (a_V1 - 2.0 * design_.airspeed_pitch_damping * w_V2) / (g.pitch_dc_gain * G);
    g.airspeed_pitch_ki = -w_V2 * w_V2 / (g.pitch_dc_gain * G);
}

void FixedWingAutopilot::reset()
{
    course_integral_ = 0.0;
    sideslip_integral_ = 0.0;
    altitude_integral_ = 0.0;
    throttle_integral_ = 0.0;
    airspeed_pitch_integral_ = 0.0;
    altitude_mode_ = AltitudeMode::HOLD;
}

real_t FixedWingAutopilot::roll_hold(real_t phi_c, real_t phi, real_t p) const
{
    return std::clamp(trim_.controls.aileron + gains_.roll_kp * (phi_c - phi) - gains_.roll_kd * p,
                      -p_.max_aileron, p_.max_aileron);
}

real_t FixedWingAutopilot::course_hold(real_t chi_c, real_t chi, real_t dt)
{
    const real_t error = std::remainder(chi_c - chi, 2.0 * consts::maths::PI);
    return limited_pi(error, gains_.course_kp, gains_.course_ki, 0.0, -design_.max_bank,
                      design_.max_bank, course_integral_, dt);
}

real_t FixedWingAutopilot::sideslip_hold(real_t beta, real_t dt)
{
    return limited_pi(-beta, gains_.sideslip_kp, gains_.sideslip_ki, trim_.controls.rudder,
                      -p_.max_rudder, p_.max_rudder, sideslip_integral_, dt);
}

real_t FixedWingAutopilot::pitch_hold(real_t theta_c, real_t theta, real_t q) const
{
    return std::clamp(trim_.controls.elevator + gains_.pitch_kp * (theta_c - theta) -
                          gains_.pitch_kd * q,
                      -p_.max_elevator, p_.max_elevator);
}

real_t FixedWingAutopilot::altitude_hold(real_t h_c, real_t h, real_t dt)
{
    return limited_pi(h_c - h, gains_.altitude_kp, gains_.altitude_ki, trim_.alpha,
                      design_.min_pitch, design_.max_pitch, altitude_integral_, dt);
}

real_t FixedWingAutopilot::airspeed_with_throttle(real_t Va_c, real_t Va, real_t dt)
{
    return limited_pi(Va_c - Va, gains_.throttle_kp, gains_.throttle_ki, trim_.controls.throttle,
                      0.0, 1.0, throttle_integral_, dt);
}

real_t FixedWingAutopilot::airspeed_with_pitch(real_t Va_c, real_t Va, real_t dt)
{
    return limited_pi(Va_c - Va, gains_.airspeed_pitch_kp, gains_.airspeed_pitch_ki, trim_.alpha,
                      design_.min_pitch, design_.max_pitch, airspeed_pitch_integral_, dt);
}

dynamics::FixedWingControls FixedWingAutopilot::update(const dynamics::FixedWingDynamics &aircraft,
                                                       const FixedWingCommands &commands, real_t dt)
{
    const auto &x = aircraft.state();
    const auto f = aircraft.forces(x, {});
    const real_t h = aircraft.altitude();

    dynamics::FixedWingControls u;

    // Lateral
    const real_t phi_c = course_hold(commands.course, aircraft.course(), dt);
    u.aileron = roll_hold(phi_c, x[fw::PHI], x[fw::P]);
    u.rudder = sideslip_hold(f.beta, dt);

    // Longitudinal
    const real_t error = commands.altitude - h;
    const AltitudeMode mode = error > design_.altitude_band    ? AltitudeMode::CLIMB
                              : error < -design_.altitude_band ? AltitudeMode::DESCEND
                                                               : AltitudeMode::HOLD;
    if (mode != altitude_mode_)
    {
        altitude_integral_ = 0.0;
        airspeed_pitch_integral_ = 0.0;
        altitude_mode_ = mode;
    }
    real_t theta_c = trim_.alpha;
    switch (mode)
    {
    case AltitudeMode::CLIMB:
        u.throttle = 1.0;
        theta_c = airspeed_with_pitch(commands.airspeed, f.airspeed, dt);
        break;
    case AltitudeMode::DESCEND:
        u.throttle = 0.0;
        theta_c = airspeed_with_pitch(commands.airspeed, f.airspeed, dt);
        break;
    case AltitudeMode::HOLD:
        theta_c = altitude_hold(commands.altitude, h, dt);
        u.throttle = airspeed_with_throttle(commands.airspeed, f.airspeed, dt);
        break;
    }
    u.elevator = pitch_hold(theta_c, x[fw::THETA], x[fw::Q]);
    return u;
}

} // namespace bitsim::control

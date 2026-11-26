#include "bitsim/dynamics/fixed_wing_dynamics.h"
#include "bitsim/bitsim_consts.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace bitsim::dynamics
{

namespace
{
const real_t PI = consts::maths::PI;
const real_t G = consts::maths::G;

// Below this airspeed the rate terms of the coefficients, which divide by it, are left out
const real_t MIN_AIRSPEED = 1.0;
// The speed over which the wheels' friction builds up: a smooth stand-in for Coulomb friction
const real_t FRICTION_SPEED = 0.1;
const real_t MAX_SUBSTEP = 0.002;

// The Euler angles' rates for the body rates
RealColVec3d euler_rates(real_t phi, real_t theta, const RealColVec3d &w)
{
    const real_t sp = std::sin(phi);
    const real_t cp = std::cos(phi);
    const real_t tt = std::tan(theta);
    const real_t ct = std::cos(theta);
    return {w[0] + sp * tt * w[1] + cp * tt * w[2], cp * w[1] - sp * w[2],
            (sp * w[1] + cp * w[2]) / ct};
}
} // namespace

FixedWingDynamics::FixedWingDynamics(const FixedWingParameters &parameters,
                                     const FixedWingState &state)
    : p_(parameters), state_(state)
{
    if (!(p_.mass > 0.0 && p_.Jx > 0.0 && p_.Jy > 0.0 && p_.Jz > 0.0 && p_.S > 0.0))
    {
        throw std::invalid_argument("FixedWingDynamics: the mass, inertias and wing area must be "
                                    "positive");
    }
    const real_t gamma = p_.Jx * p_.Jz - p_.Jxz * p_.Jxz;
    g_[0] = gamma;
    g_[1] = p_.Jxz * (p_.Jx - p_.Jy + p_.Jz) / gamma;
    g_[2] = (p_.Jz * (p_.Jz - p_.Jy) + p_.Jxz * p_.Jxz) / gamma;
    g_[3] = p_.Jz / gamma;
    g_[4] = p_.Jxz / gamma;
    g_[5] = (p_.Jz - p_.Jx) / p_.Jy;
    g_[6] = p_.Jxz / p_.Jy;
    g_[7] = ((p_.Jx - p_.Jy) * p_.Jx + p_.Jxz * p_.Jxz) / gamma;
    g_[8] = p_.Jx / gamma;
}

RealMat3d FixedWingDynamics::rotation(real_t phi, real_t theta, real_t psi)
{
    return (Eigen::AngleAxisd(psi, RealColVec3d::UnitZ()) *
            Eigen::AngleAxisd(theta, RealColVec3d::UnitY()) *
            Eigen::AngleAxisd(phi, RealColVec3d::UnitX()))
        .toRotationMatrix();
}

FixedWingControls FixedWingDynamics::limit(const FixedWingControls &c) const
{
    FixedWingControls out;
    out.elevator = std::clamp(c.elevator, -p_.max_elevator, p_.max_elevator);
    out.aileron = std::clamp(c.aileron, -p_.max_aileron, p_.max_aileron);
    out.rudder = std::clamp(c.rudder, -p_.max_rudder, p_.max_rudder);
    out.throttle = std::clamp(c.throttle, 0.0, p_.max_throttle);
    out.brake = std::clamp(c.brake, 0.0, 1.0);
    return out;
}

void FixedWingDynamics::propeller_(real_t Va, real_t throttle, real_t &thrust, real_t &torque) const
{
    // The motor speed where the motor's torque, KQ (V - KQ Omega) / R - KQ i0, equals the
    // propeller's, rho D^5 / (2 pi)^2 CQ(J) Omega^2 with the advance ratio J = 2 pi Va / (Omega D)
    const real_t D = p_.prop_diameter;
    const real_t KQ = 60.0 / (2.0 * PI * p_.motor_KV);
    const real_t V = p_.battery_V * throttle;
    const real_t a = p_.rho * std::pow(D, 5) / ((2.0 * PI) * (2.0 * PI)) * p_.CQ0;
    const real_t b = p_.rho * std::pow(D, 4) / (2.0 * PI) * p_.CQ1 * Va + KQ * KQ / p_.motor_R;
    const real_t c =
        p_.rho * std::pow(D, 3) * p_.CQ2 * Va * Va - KQ * V / p_.motor_R + KQ * p_.motor_i0;
    const real_t disc = std::max(b * b - 4.0 * a * c, 0.0);
    const real_t omega = std::max((-b + std::sqrt(disc)) / (2.0 * a), 0.0);
    if (omega < 1.0e-6)
    {
        thrust = 0.0;
        torque = 0.0;
        return;
    }
    const real_t J = 2.0 * PI * Va / (omega * D);
    const real_t CT = p_.CT2 * J * J + p_.CT1 * J + p_.CT0;
    const real_t CQ = p_.CQ2 * J * J + p_.CQ1 * J + p_.CQ0;
    const real_t n = omega / (2.0 * PI);
    // Negative when the air turns the propeller faster than the motor: it windmills, a drag
    thrust = p_.rho * n * n * std::pow(D, 4) * CT;
    torque = p_.rho * n * n * std::pow(D, 5) * CQ;
}

FixedWingForces FixedWingDynamics::forces(const FixedWingState &x,
                                          const FixedWingControls &controls) const
{
    const FixedWingControls u = limit(controls);
    const RealMat3d R = rotation(x[fw::PHI], x[fw::THETA], x[fw::PSI]);
    const RealColVec3d vel(x[fw::U], x[fw::V], x[fw::W]);
    const RealColVec3d rates(x[fw::P], x[fw::Q], x[fw::R]);

    FixedWingForces out;
    out.force = R.transpose() * RealColVec3d(0.0, 0.0, p_.mass * G);
    out.moment.setZero();

    // Air data
    const RealColVec3d vr = vel - R.transpose() * wind_;
    const real_t Va = vr.norm();
    out.airspeed = Va;
    out.alpha = std::atan2(vr[2], vr[0]);
    out.beta = Va > 1.0e-9 ? std::asin(std::clamp(vr[1] / Va, -1.0, 1.0)) : 0.0;
    const real_t alpha = out.alpha;
    const real_t beta = out.beta;

    // Aerodynamics
    const real_t qbar = 0.5 * p_.rho * Va * Va;
    const bool rates_count = Va > MIN_AIRSPEED;
    const real_t pn = rates_count ? p_.b * rates[0] / (2.0 * Va) : 0.0;
    const real_t qn = rates_count ? p_.c * rates[1] / (2.0 * Va) : 0.0;
    const real_t rn = rates_count ? p_.b * rates[2] / (2.0 * Va) : 0.0;

    const real_t M = p_.stall_sharpness;
    const real_t a0 = p_.stall_alpha;
    const real_t e1 = std::exp(std::clamp(-M * (alpha - a0), -50.0, 50.0));
    const real_t e2 = std::exp(std::clamp(M * (alpha + a0), -50.0, 50.0));
    const real_t sigma = (1.0 + e1 + e2) / ((1.0 + e1) * (1.0 + e2));
    const real_t CL_linear = p_.CL0 + p_.CL_alpha * alpha;
    const real_t sa = std::sin(alpha);
    const real_t ca = std::cos(alpha);
    const real_t CL_alpha =
        (1.0 - sigma) * CL_linear + sigma * 2.0 * (alpha >= 0.0 ? 1.0 : -1.0) * sa * sa * ca;
    const real_t CD_alpha = p_.CD0 + CL_linear * CL_linear / (PI * p_.e * p_.aspect_ratio());

    const real_t lift = qbar * p_.S * (CL_alpha + p_.CL_q * qn + p_.CL_de * u.elevator);
    const real_t drag = qbar * p_.S * (CD_alpha + p_.CD_q * qn + p_.CD_de * u.elevator);
    const real_t side = qbar * p_.S *
                        (p_.CY0 + p_.CY_beta * beta + p_.CY_p * pn + p_.CY_r * rn +
                         p_.CY_da * u.aileron + p_.CY_dr * u.rudder);
    // From the stability frame, whose x is along the air, to the body
    out.force += RealColVec3d(-drag * ca + lift * sa, side, -drag * sa - lift * ca);

    out.moment[0] = qbar * p_.S * p_.b *
                    (p_.Cl0 + p_.Cl_beta * beta + p_.Cl_p * pn + p_.Cl_r * rn +
                     p_.Cl_da * u.aileron + p_.Cl_dr * u.rudder);
    out.moment[1] =
        qbar * p_.S * p_.c * (p_.Cm0 + p_.Cm_alpha * alpha + p_.Cm_q * qn + p_.Cm_de * u.elevator);
    out.moment[2] = qbar * p_.S * p_.b *
                    (p_.Cn0 + p_.Cn_beta * beta + p_.Cn_p * pn + p_.Cn_r * rn +
                     p_.Cn_da * u.aileron + p_.Cn_dr * u.rudder);

    // The propeller pushes along x; the motor turns it one way and the aircraft the other
    real_t thrust = 0.0;
    real_t torque = 0.0;
    propeller_(std::max(vr[0], 0.0), u.throttle, thrust, torque);
    out.thrust = thrust;
    out.force[0] += thrust;
    out.moment[0] -= torque;

    // The landing gear on the ground at down = 0
    for (const auto &wheel : p_.gear)
    {
        const RealColVec3d r(wheel.position[0], wheel.position[1], wheel.position[2]);
        const RealColVec3d at = RealColVec3d(x[fw::PN], x[fw::PE], x[fw::PD]) + R * r;
        const real_t depth = at[2];
        if (depth <= 0.0)
        {
            continue;
        }
        const RealColVec3d v_point = R * (vel + rates.cross(r));
        const real_t normal = std::max(wheel.stiffness * depth + wheel.damping * v_point[2], 0.0);
        out.ground_load += normal;

        // The wheel's heading on the ground, turned by the nose wheel's steering
        const real_t heading = x[fw::PSI] - (wheel.steers ? p_.nose_steering * u.rudder : 0.0);
        const RealColVec3d along(std::cos(heading), std::sin(heading), 0.0);
        const RealColVec3d across(-std::sin(heading), std::cos(heading), 0.0);
        const real_t v_along = v_point.dot(along);
        const real_t v_across = v_point.dot(across);
        const real_t roll_mu =
            p_.rolling_friction + (wheel.brakes ? p_.brake_friction * u.brake : 0.0);
        const RealColVec3d f_world =
            RealColVec3d(0.0, 0.0, -normal) -
            along * (roll_mu * normal * std::tanh(v_along / FRICTION_SPEED)) -
            across * (p_.side_friction * normal * std::tanh(v_across / FRICTION_SPEED));
        const RealColVec3d f_body = R.transpose() * f_world;
        out.force += f_body;
        out.moment += r.cross(f_body);
    }
    return out;
}

FixedWingState FixedWingDynamics::derivative(const FixedWingState &x,
                                             const FixedWingControls &controls) const
{
    const FixedWingForces f = forces(x, controls);
    const RealMat3d R = rotation(x[fw::PHI], x[fw::THETA], x[fw::PSI]);
    const RealColVec3d vel(x[fw::U], x[fw::V], x[fw::W]);
    const real_t p = x[fw::P];
    const real_t q = x[fw::Q];
    const real_t r = x[fw::R];

    FixedWingState dx;
    dx.segment<3>(fw::PN) = R * vel;
    dx[fw::U] = r * vel[1] - q * vel[2] + f.force[0] / p_.mass;
    dx[fw::V] = p * vel[2] - r * vel[0] + f.force[1] / p_.mass;
    dx[fw::W] = q * vel[0] - p * vel[1] + f.force[2] / p_.mass;
    dx.segment<3>(fw::PHI) = euler_rates(x[fw::PHI], x[fw::THETA], {p, q, r});
    const RealColVec3d &m = f.moment;
    dx[fw::P] = g_[1] * p * q - g_[2] * q * r + g_[3] * m[0] + g_[4] * m[2];
    dx[fw::Q] = g_[5] * p * r - g_[6] * (p * p - r * r) + m[1] / p_.Jy;
    dx[fw::R] = g_[7] * p * q - g_[1] * q * r + g_[4] * m[0] + g_[8] * m[2];
    return dx;
}

void FixedWingDynamics::step(const FixedWingControls &controls, real_t dt)
{
    if (!(dt > 0.0))
    {
        throw std::invalid_argument("FixedWingDynamics: dt must be positive");
    }
    const FixedWingControls u = limit(controls);
    const int n = static_cast<int>(std::ceil(dt / MAX_SUBSTEP - 1.0e-9));
    const real_t h = dt / n;
    for (int i = 0; i < n; ++i)
    {
        const FixedWingState k1 = derivative(state_, u);
        const FixedWingState k2 = derivative(state_ + 0.5 * h * k1, u);
        const FixedWingState k3 = derivative(state_ + 0.5 * h * k2, u);
        const FixedWingState k4 = derivative(state_ + h * k3, u);
        state_ += h / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
        state_[fw::PSI] = std::remainder(state_[fw::PSI], 2.0 * PI);
        state_[fw::PHI] = std::remainder(state_[fw::PHI], 2.0 * PI);
    }
    time_ += dt;
}

real_t FixedWingDynamics::airspeed() const { return forces(state_, {}).airspeed; }
real_t FixedWingDynamics::alpha() const { return forces(state_, {}).alpha; }
real_t FixedWingDynamics::beta() const { return forces(state_, {}).beta; }

real_t FixedWingDynamics::ground_speed() const
{
    const RealColVec3d v = rotation(state_[fw::PHI], state_[fw::THETA], state_[fw::PSI]) *
                           RealColVec3d(state_[fw::U], state_[fw::V], state_[fw::W]);
    return std::hypot(v[0], v[1]);
}

real_t FixedWingDynamics::course() const
{
    const RealColVec3d v = rotation(state_[fw::PHI], state_[fw::THETA], state_[fw::PSI]) *
                           RealColVec3d(state_[fw::U], state_[fw::V], state_[fw::W]);
    return std::hypot(v[0], v[1]) > 0.1 ? std::atan2(v[1], v[0]) : state_[fw::PSI];
}

real_t FixedWingDynamics::climb_rate() const
{
    const RealColVec3d v = rotation(state_[fw::PHI], state_[fw::THETA], state_[fw::PSI]) *
                           RealColVec3d(state_[fw::U], state_[fw::V], state_[fw::W]);
    return -v[2];
}

bool FixedWingDynamics::on_ground() const { return forces(state_, {}).ground_load > 0.0; }

} // namespace bitsim::dynamics

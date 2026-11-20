#include "bitrl/dynamics/fixed_wing_trim.h"
#include "bitrl/bitrl_consts.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace bitrl::dynamics
{

namespace
{
using Vector6 = Eigen::Matrix<real_t, 6, 1>;

// The trim unknowns: alpha, beta, elevator, aileron, rudder, throttle
FixedWingState trim_state(const Vector6 &z, real_t Va, real_t altitude, real_t heading)
{
    FixedWingState x = FixedWingState::Zero();
    x[fw::PD] = -altitude;
    x[fw::U] = Va * std::cos(z[0]) * std::cos(z[1]);
    x[fw::V] = Va * std::sin(z[1]);
    x[fw::W] = Va * std::sin(z[0]) * std::cos(z[1]);
    x[fw::THETA] = z[0];
    x[fw::PSI] = heading;
    return x;
}

FixedWingControls trim_controls(const Vector6 &z)
{
    FixedWingControls u;
    u.elevator = z[2];
    u.aileron = z[3];
    u.rudder = z[4];
    u.throttle = z[5];
    return u;
}

// The accelerations: they are all 0 in a trim
Vector6 trim_residual(const FixedWingDynamics &model, const Vector6 &z, real_t Va, real_t altitude,
                      real_t heading)
{
    const FixedWingState dx =
        model.derivative(trim_state(z, Va, altitude, heading), trim_controls(z));
    Vector6 r;
    r << dx[fw::U], dx[fw::V], dx[fw::W], dx[fw::P], dx[fw::Q], dx[fw::R];
    return r;
}

FixedWingControls perturbed(FixedWingControls u, int i, real_t h)
{
    switch (i)
    {
    case 0:
        u.elevator += h;
        break;
    case 1:
        u.aileron += h;
        break;
    case 2:
        u.rudder += h;
        break;
    default:
        u.throttle += h;
        break;
    }
    return u;
}

FlightMode make_mode(const std::string &name, const std::complex<real_t> &l)
{
    FlightMode m;
    m.name = name;
    m.eigenvalue = l;
    m.natural_frequency = std::abs(l);
    m.damping = m.natural_frequency > 0.0 ? -l.real() / m.natural_frequency : 0.0;
    m.period = std::abs(l.imag()) > 1.0e-9 ? 2.0 * consts::maths::PI / std::abs(l.imag()) : 0.0;
    m.time_constant = std::abs(l.real()) > 1.0e-12 ? -1.0 / l.real() : 0.0;
    return m;
}

// The eigenvalues of the submatrix of A on the given states, with one of each complex pair
std::vector<std::complex<real_t>>
eigenvalues(const Eigen::Matrix<real_t, 12, 12, Eigen::DontAlign> &A,
            const std::array<int, 4> &states)
{
    Eigen::Matrix4d sub;
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            sub(i, j) = A(states[i], states[j]);
        }
    }
    Eigen::EigenSolver<Eigen::Matrix4d> solver(sub);
    std::vector<std::complex<real_t>> out;
    for (int i = 0; i < 4; ++i)
    {
        const auto l = solver.eigenvalues()[i];
        if (l.imag() >= 0.0)
        {
            out.push_back(l);
        }
    }
    return out;
}
} // namespace

FixedWingTrim trim_level_flight(const FixedWingParameters &parameters, real_t airspeed,
                                real_t altitude, real_t heading)
{
    if (!(airspeed > 0.0))
    {
        throw std::invalid_argument("trim_level_flight: the airspeed must be positive");
    }
    // Search without the limits of the controls, and check them at the end: limited controls
    // would make the search stall where a control stops having any effect
    FixedWingParameters unlimited = parameters;
    unlimited.max_elevator = unlimited.max_aileron = unlimited.max_rudder = 1.0e3;
    unlimited.max_throttle = 1.0e3;
    const FixedWingDynamics model(unlimited, FixedWingState::Zero());

    // Start from the angle of attack whose lift carries the weight and the elevator that
    // balances its pitching moment
    const real_t CL = 2.0 * parameters.mass * consts::maths::G /
                      (parameters.rho * airspeed * airspeed * parameters.S);
    Vector6 z;
    z[0] = (CL - parameters.CL0) / parameters.CL_alpha;
    z[1] = 0.0;
    z[2] = -(parameters.Cm0 + parameters.Cm_alpha * z[0]) / parameters.Cm_de;
    z[3] = 0.0;
    z[4] = 0.0;
    z[5] = 0.5;

    // Levenberg-Marquardt with a numerical Jacobian
    Vector6 r = trim_residual(model, z, airspeed, altitude, heading);
    real_t lambda = 1.0e-3;
    for (int iteration = 0; iteration < 200 && r.cwiseAbs().maxCoeff() > 1.0e-12; ++iteration)
    {
        Eigen::Matrix<real_t, 6, 6> J;
        const real_t h = 1.0e-7;
        for (int j = 0; j < 6; ++j)
        {
            Vector6 plus = z;
            Vector6 minus = z;
            plus[j] += h;
            minus[j] -= h;
            J.col(j) = (trim_residual(model, plus, airspeed, altitude, heading) -
                        trim_residual(model, minus, airspeed, altitude, heading)) /
                       (2.0 * h);
        }
        const Eigen::Matrix<real_t, 6, 6> JtJ = J.transpose() * J;
        const Vector6 g = J.transpose() * r;
        bool improved = false;
        for (int tries = 0; tries < 20 && !improved; ++tries)
        {
            Eigen::Matrix<real_t, 6, 6> M = JtJ;
            M.diagonal() += lambda * (JtJ.diagonal().array() + 1.0e-12).matrix();
            const Vector6 next = z - M.ldlt().solve(g);
            const Vector6 r_next = trim_residual(model, next, airspeed, altitude, heading);
            if (r_next.norm() < r.norm())
            {
                z = next;
                r = r_next;
                lambda = std::max(lambda / 10.0, 1.0e-12);
                improved = true;
            }
            else
            {
                lambda *= 10.0;
            }
        }
        if (!improved)
        {
            break;
        }
    }

    FixedWingTrim trim;
    trim.state = trim_state(z, airspeed, altitude, heading);
    trim.controls = trim_controls(z);
    trim.airspeed = airspeed;
    trim.alpha = z[0];
    trim.beta = z[1];
    trim.residual = r.cwiseAbs().maxCoeff();
    // Converged, and the controls within the aircraft's limits
    const auto &c = trim.controls;
    trim.converged = trim.residual < 1.0e-8 && std::abs(c.elevator) <= parameters.max_elevator &&
                     std::abs(c.aileron) <= parameters.max_aileron &&
                     std::abs(c.rudder) <= parameters.max_rudder && c.throttle >= 0.0 &&
                     c.throttle <= parameters.max_throttle;
    return trim;
}

FixedWingLinearModel linearise(const FixedWingParameters &parameters, const FixedWingTrim &trim)
{
    const FixedWingDynamics model(parameters, trim.state);
    FixedWingLinearModel lin;
    const real_t h = 1.0e-6;
    for (int j = 0; j < 12; ++j)
    {
        FixedWingState plus = trim.state;
        FixedWingState minus = trim.state;
        plus[j] += h;
        minus[j] -= h;
        lin.A.col(j) =
            (model.derivative(plus, trim.controls) - model.derivative(minus, trim.controls)) /
            (2.0 * h);
    }
    for (int j = 0; j < 4; ++j)
    {
        lin.B.col(j) = (model.derivative(trim.state, perturbed(trim.controls, j, h)) -
                        model.derivative(trim.state, perturbed(trim.controls, j, -h))) /
                       (2.0 * h);
    }
    return lin;
}

std::vector<FlightMode> flight_modes(const FixedWingLinearModel &model)
{
    std::vector<FlightMode> modes;

    // Longitudinal: two oscillations, the faster the short period
    auto lon = eigenvalues(model.A, {fw::U, fw::W, fw::Q, fw::THETA});
    std::sort(lon.begin(), lon.end(),
              [](const auto &a, const auto &b) { return std::abs(a) > std::abs(b); });
    const char *lon_names[] = {"short period", "phugoid"};
    for (std::size_t i = 0; i < lon.size() && i < 2; ++i)
    {
        modes.push_back(make_mode(lon_names[i], lon[i]));
    }
    for (std::size_t i = 2; i < lon.size(); ++i)
    {
        modes.push_back(make_mode("longitudinal", lon[i]));
    }

    // Lateral: the Dutch roll oscillates, the roll mode is the fast real one and the spiral the
    // slow one
    auto lat = eigenvalues(model.A, {fw::V, fw::P, fw::R, fw::PHI});
    std::vector<std::complex<real_t>> real_ones;
    for (const auto &l : lat)
    {
        if (std::abs(l.imag()) > 1.0e-9)
        {
            modes.push_back(make_mode("Dutch roll", l));
        }
        else
        {
            real_ones.push_back(l);
        }
    }
    std::sort(real_ones.begin(), real_ones.end(),
              [](const auto &a, const auto &b) { return std::abs(a) > std::abs(b); });
    const char *lat_names[] = {"roll", "spiral"};
    for (std::size_t i = 0; i < real_ones.size(); ++i)
    {
        modes.push_back(make_mode(i < 2 ? lat_names[i] : "lateral", real_ones[i]));
    }
    return modes;
}

} // namespace bitrl::dynamics

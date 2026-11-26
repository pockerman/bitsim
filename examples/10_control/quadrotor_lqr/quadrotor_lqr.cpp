// A quadrotor flown along a path and pushed off it, by an LQR and by the flight controller of
// quadrotor_rc_simulation with a position loop around it.
//
// Usage: quadrotor_lqr [--render] [--record DIR]
//
// The LQR is designed on the model linearised at hover. The other drone flies with the angle mode
// flight controller, whose sticks a PD position loop moves: a cascade of the kind most drones
// fly with. Both follow the same reference along a square, 2.5 m apart, and get the same push
// sideways. --render shows them in a window, in real time; --record DIR also writes the window to
// DIR, ten pictures a second, which needs bitsim built with Chrono and Irrlicht.
#include "bitsim/bitsim_config.h"
#include "bitsim/bitsim_consts.h"
#include "bitsim/bitsim_types.h"
#include "bitsim/control/lqr.h"
#include "bitsim/control/quadrotor_flight_controller.h"
#include "bitsim/control/rc_command.h"
#include "bitsim/dynamics/quadrotor_dynamics.h"
#include "bitsim/dynamics/system_state.h"
#include "bitsim/geometry/rotations.h"

#if defined(BITSIM_IRRLICHT) && defined(BITSIM_CHRONO)
#include "bitsim/utils/render/quadrotor_view.h"
#endif

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace quadrotor_lqr
{

using bitsim::index_t;
using bitsim::real_t;
using bitsim::RealColVec3d;
using bitsim::RealVec;
using bitsim::uint_t;
using bitsim::control::Matrix;
using bitsim::control::QuadrotorFlightController;
using bitsim::control::RCCommand;
using bitsim::control::Vector;
using bitsim::dynamics::QuadrotorDynamics;
using bitsim::dynamics::QuadrotorDynamicsConfig;
using bitsim::dynamics::SysState;

const real_t G = bitsim::consts::maths::G;
const real_t STEP_SIZE = 0.001;     // [s] of the dynamics
const real_t CONTROL_PERIOD = 0.01; // [s] both controllers run at 100 Hz
const uint_t SUB_STEPS = 10;        // dynamics steps per control period
const real_t SPEED = 1.0;           // of the reference along the path [m/s]
const real_t START = 2.0;           // when the reference starts to move [s]
const real_t PUSH_AT = 22.0;        // [s]
const real_t PUSH = 2.0;            // the push, sideways to the east [m/s]
const real_t DURATION = 28.0;       // [s]
const real_t SEPARATION = 2.5;      // the cascade flies 2.5 m west of the LQR [m]
const real_t PRINT_PERIOD = 2.0;    // [s]
const real_t RENDER_RATE = 30.0;    // [frames/s]
const real_t RECORD_RATE = 10.0;    // [frames/s]

// The names of the state of QuadrotorDynamics, in the order of the LQR's state vector
const std::array<const char *, 12> NAMES{"x", "y", "z", "u",   "v",     "w",
                                         "p", "q", "r", "phi", "theta", "psi"};

// The quadrotor of quadrotor_rc_simulation
QuadrotorDynamicsConfig quadrotor_config()
{
    QuadrotorDynamicsConfig config;
    config.dt = STEP_SIZE;
    config.mass = 0.468;
    config.l = 0.225;
    config.k_1 = 2.980e-6;
    config.k_2 = 1.140e-7;
    config.Jx = 4.856e-3;
    config.Jy = 4.856e-3;
    config.Jz = 8.801e-3;
    config.drag = 0.25;
    return config;
}

SysState<12> to_state(const Vector &x)
{
    std::array<std::pair<std::string, real_t>, 12> values;
    for (uint_t i = 0; i < 12; ++i)
    {
        values[i] = {NAMES[i], x[static_cast<index_t>(i)]};
    }
    return SysState<12>(std::move(values));
}

Vector to_vector(const SysState<12> &state)
{
    Vector x(12);
    for (uint_t i = 0; i < 12; ++i)
    {
        x[static_cast<index_t>(i)] = state.get(NAMES[i]);
    }
    return x;
}

// The rotor speeds for a thrust and the torques, u = [T, tau_phi, tau_theta, tau_psi]
RealVec mix(const QuadrotorDynamicsConfig &config, const Vector &u, real_t max_rotor_speed)
{
    return bitsim::control::quadrotor_mixer(config, u[0], RealColVec3d(u[1], u[2], u[3]),
                                            max_rotor_speed);
}

// The state one control period on, from x with u held
Vector step(const QuadrotorDynamicsConfig &config, const Vector &x, const Vector &u,
            real_t max_rotor_speed)
{
    SysState<12> state = to_state(x);
    QuadrotorDynamics dynamics(config, state);
    const RealVec rotors = mix(config, u, max_rotor_speed);
    for (uint_t k = 0; k < SUB_STEPS; ++k)
    {
        dynamics.integrate(rotors);
    }
    return to_vector(dynamics.get_state());
}

// The map of one control period linearised about hover, by central differences: the discrete
// model x_{k+1} = A x_k + B u_k of the deviations from hover
bitsim::control::DiscreteLinearModel linearise(const QuadrotorDynamicsConfig &config,
                                               const Vector &hover, const Vector &u_hover,
                                               real_t max_rotor_speed)
{
    bitsim::control::DiscreteLinearModel model{Matrix(12, 12), Matrix(12, 4)};
    const real_t h = 1.0e-5;
    for (index_t j = 0; j < 12; ++j)
    {
        Vector plus = hover;
        Vector minus = hover;
        plus[j] += h;
        minus[j] -= h;
        model.A.col(j) = (step(config, plus, u_hover, max_rotor_speed) -
                          step(config, minus, u_hover, max_rotor_speed)) /
                         (2.0 * h);
    }
    for (index_t j = 0; j < 4; ++j)
    {
        Vector plus = u_hover;
        Vector minus = u_hover;
        const real_t hu = h * std::max(1.0, std::abs(u_hover[j]));
        plus[j] += hu;
        minus[j] -= hu;
        model.B.col(j) = (step(config, hover, plus, max_rotor_speed) -
                          step(config, hover, minus, max_rotor_speed)) /
                         (2.0 * hu);
    }
    return model;
}

// The reference: it waits at the start, then moves along a square at SPEED, and stops at the end
struct Path
{
    std::vector<RealColVec3d> corners{{0, 0, -2}, {4, 0, -2}, {4, 4, -3}, {0, 4, -3}, {0, 0, -2}};

    RealColVec3d at(real_t time) const
    {
        real_t s = std::max(0.0, time - START) * SPEED;
        for (uint_t i = 0; i + 1 < corners.size(); ++i)
        {
            const RealColVec3d leg = corners[i + 1] - corners[i];
            if (s <= leg.norm())
            {
                return corners[i] + leg * (s / leg.norm());
            }
            s -= leg.norm();
        }
        return corners.back();
    }

    // The velocity of the reference, by a central difference
    RealColVec3d velocity(real_t time) const
    {
        const real_t h = 1.0e-3;
        return (at(time + h) - at(std::max(0.0, time - h))) / (time + h - std::max(0.0, time - h));
    }
};

// A drone and what happened to it
struct Drone
{
    std::string name;
    RealColVec3d offset; // its path is the reference moved by this
    SysState<12> state;
    QuadrotorDynamics dynamics;
    real_t sum_sq_error{0.0};
    uint_t n_errors{0};
    real_t max_error_after_push{0.0};
    real_t back_within_10cm{-1.0};

    Drone(const std::string &n, const RealColVec3d &o, const QuadrotorDynamicsConfig &config,
          const Vector &x0)
        : name(n), offset(o), state(to_state(x0)), dynamics(config, state)
    {
    }

    RealColVec3d position() const { return dynamics.get_position(); }

    // The velocity in the world, NED
    RealColVec3d world_velocity() const
    {
        const auto R = bitsim::geometry::from_yaw_pitch_roll(
            {dynamics.psi(), dynamics.theta(), dynamics.phi()});
        return R * dynamics.get_velocity();
    }

    // A push: a change of the velocity in the world, east
    void push(real_t east)
    {
        const auto R = bitsim::geometry::from_yaw_pitch_roll(
            {dynamics.psi(), dynamics.theta(), dynamics.phi()});
        const RealColVec3d v = dynamics.get_velocity() + R.transpose() * RealColVec3d(0, east, 0);
        dynamics.get_state().set("u", v[0]);
        dynamics.get_state().set("v", v[1]);
        dynamics.get_state().set("w", v[2]);
    }

    void record_error(const RealColVec3d &reference, real_t time)
    {
        const real_t e = (position() - reference).norm();
        if (time < PUSH_AT)
        {
            sum_sq_error += e * e;
            ++n_errors;
        }
        else
        {
            max_error_after_push = std::max(max_error_after_push, e);
            if (e > 0.1)
            {
                back_within_10cm = -1.0;
            }
            else if (back_within_10cm < 0.0)
            {
                back_within_10cm = time - PUSH_AT;
            }
        }
    }
};

// The position loop of the cascade: from where the drone is and where it should be, the sticks
// of the angle mode flight controller. A PD loop sets the acceleration it wants, and the tilt and
// the thrust that give it
RCCommand position_loop(const Drone &drone, const RealColVec3d &target,
                        const RealColVec3d &target_velocity,
                        const QuadrotorFlightController &controller)
{
    const real_t kp = 1.0; // [1/s^2]
    const real_t kd = 1.6; // [1/s]
    const real_t kp_z = 2.0;
    const real_t kd_z = 2.4;
    const RealColVec3d e = target - drone.position();
    const RealColVec3d v = drone.world_velocity() - target_velocity;

    // the horizontal acceleration it wants, north and east, and the climb, up
    const real_t a_north = kp * e[0] - kd * v[0];
    const real_t a_east = kp * e[1] - kd * v[1];
    const real_t a_up = -kp_z * e[2] + kd_z * v[2];

    const real_t max_tilt = controller.parameters().max_tilt;
    const auto &d = drone.dynamics;
    RCCommand command;
    command.armed = true;
    // tilting forward by theta gives g tan(theta) forward; pitch forward is nose down
    command.pitch = std::atan(a_north / G) / max_tilt;
    command.roll = std::atan(a_east / G) / max_tilt;
    command.throttle =
        controller.hover_throttle() * (1.0 + a_up / G) / (std::cos(d.phi()) * std::cos(d.theta()));
    return command.clamped();
}

struct Options
{
    bool render{false};
    std::string record;
};

int run(const Options &options)
{
    const QuadrotorDynamicsConfig config = quadrotor_config();
    const QuadrotorFlightController flight_controller(config);
    const real_t max_rotor_speed = flight_controller.max_rotor_speed();

    // Hover: level, still, the thrust carrying the weight
    const Path path;
    Vector hover = Vector::Zero(12);
    hover.head(3) = path.at(0.0);
    Vector u_hover = Vector::Zero(4);
    u_hover[0] = config.mass * G;

    const auto model = linearise(config, hover, u_hover, max_rotor_speed);

    // Bryson's rule: each weight is one over the square of how much is acceptable: 0.5 m of
    // position, 1 m/s, 2 rad/s, 0.3 rad of tilt; 2 N of thrust and 0.05 N m of torque
    Vector q(12);
    q << 4, 4, 4, 1, 1, 1, 0.25, 0.25, 0.25, 11, 11, 11;
    Vector r(4);
    r << 0.25, 400, 400, 400;
    const auto lqr = bitsim::control::dlqr(model.A, model.B, q.asDiagonal().toDenseMatrix(),
                                           r.asDiagonal().toDenseMatrix());
    const Matrix A_closed = model.A - model.B * lqr.K;
    std::printf("LQR designed at hover: the slowest closed loop mode decays by %.4f a step, %.2f "
                "s to halve\n",
                A_closed.eigenvalues().cwiseAbs().maxCoeff(),
                std::log(0.5) / std::log(A_closed.eigenvalues().cwiseAbs().maxCoeff()) *
                    CONTROL_PERIOD);

    Drone lqr_drone("LQR", RealColVec3d::Zero(), config, hover);
    Vector cascade_start = hover;
    cascade_start[1] -= SEPARATION;
    Drone cascade_drone("cascade", RealColVec3d(0, -SEPARATION, 0), config, cascade_start);

#if defined(BITSIM_IRRLICHT) && defined(BITSIM_CHRONO)
    using bitsim::utils::render::QuadrotorView;
    std::optional<QuadrotorView> view;
    if (options.render || !options.record.empty())
    {
        QuadrotorView::Style lqr_style;
        lqr_style.body = chrono::ChColor(0.25f, 0.45f, 0.8f);
        lqr_style.trail = chrono::ChColor(0.3f, 0.6f, 1.0f);
        QuadrotorView::Style cascade_style;
        cascade_style.body = chrono::ChColor(0.85f, 0.45f, 0.1f);
        cascade_style.trail = chrono::ChColor(1.0f, 0.8f, 0.2f);
        view.emplace("A quadrotor flown by an LQR, and by a cascade of PD loops",
                     std::vector<QuadrotorView::Style>{lqr_style, cascade_style}, options.record,
                     RECORD_RATE);
        view->fix_camera(RealColVec3d(-2.5, -5.5, -5.0), RealColVec3d(2.0, 0.8, -1.2));
    }
#else
    if (options.render || !options.record.empty())
    {
        std::cerr << "--render and --record need bitsim built with Chrono and Irrlicht\n";
        return 1;
    }
#endif

    std::printf("\n time [s]   reference          LQR error [m]   cascade error [m]\n");
    const auto steps = static_cast<uint_t>(std::lround(DURATION / CONTROL_PERIOD));
    real_t next_print = 0.0;
    real_t next_render = 0.0;
    const auto wall_start = std::chrono::steady_clock::now();
    for (uint_t k = 0; k <= steps; ++k)
    {
        const real_t time = k * CONTROL_PERIOD;
        if (std::abs(time - PUSH_AT) < 0.5 * CONTROL_PERIOD)
        {
            lqr_drone.push(PUSH);
            cascade_drone.push(PUSH);
        }
        const RealColVec3d reference = path.at(time);
        lqr_drone.record_error(reference, time);
        cascade_drone.record_error(reference + cascade_drone.offset, time);

        if (time >= next_print - 1e-9)
        {
            std::printf("%9.1f   (%4.1f %4.1f %4.1f)   %13.3f   %17.3f\n", time, reference[0],
                        reference[1], -reference[2], (lqr_drone.position() - reference).norm(),
                        (cascade_drone.position() - reference - cascade_drone.offset).norm());
            next_print += PRINT_PERIOD;
        }

#if defined(BITSIM_IRRLICHT) && defined(BITSIM_CHRONO)
        if (view && time >= next_render - 1e-9)
        {
            view->set_targets({reference, reference + cascade_drone.offset});
            char status[160];
            std::snprintf(status, sizeof(status),
                          "t %5.1f s   blue LQR: off by %.2f m   orange cascade: off by %.2f m%s",
                          time, (lqr_drone.position() - reference).norm(),
                          (cascade_drone.position() - reference - cascade_drone.offset).norm(),
                          (time >= PUSH_AT && time < PUSH_AT + 1.5) ? "   PUSH" : "");
            if (!view->show({&lqr_drone.dynamics, &cascade_drone.dynamics}, time, status))
            {
                break;
            }
            next_render += 1.0 / RENDER_RATE;
            if (options.render)
            {
                // in real time
                const real_t ahead = time - std::chrono::duration<real_t>(
                                                std::chrono::steady_clock::now() - wall_start)
                                                .count();
                if (ahead > 0.0)
                {
                    std::this_thread::sleep_for(std::chrono::duration<real_t>(ahead));
                }
            }
        }
#endif

        // the LQR: u = u_hover - K (x - x_ref), with x_ref hover at the reference, moving with
        // it: level and at the reference's velocity, which in the body frame of a level drone
        // facing north is the same
        const RealColVec3d reference_velocity = path.velocity(time);
        Vector x_ref = hover;
        x_ref.head(3) = reference;
        x_ref.segment(3, 3) = reference_velocity;
        const Vector x = to_vector(lqr_drone.dynamics.get_state());
        const Vector u = u_hover - lqr.K * (x - x_ref);
        const RealVec lqr_rotors = mix(config, u, max_rotor_speed);

        // the cascade: the position loop sets the sticks, the flight controller the rotors
        const RCCommand command = position_loop(cascade_drone, reference + cascade_drone.offset,
                                                reference_velocity, flight_controller);
        for (uint_t s = 0; s < SUB_STEPS; ++s)
        {
            lqr_drone.dynamics.integrate(lqr_rotors);
            cascade_drone.dynamics.integrate(
                flight_controller.rotor_speeds(command, cascade_drone.dynamics));
        }
    }

    std::printf("\n%-8s  RMS error along the path [m]   largest error after the push [m]   back "
                "within 0.1 m after [s]\n",
                "");
    for (const Drone *d : {&lqr_drone, &cascade_drone})
    {
        std::printf("%-8s  %28.3f   %32.3f   %26.2f\n", d->name.c_str(),
                    std::sqrt(d->sum_sq_error / std::max<uint_t>(1, d->n_errors)),
                    d->max_error_after_push, d->back_within_10cm);
    }

    // what the example shows: the LQR follows the path and recovers from the push
    const bool ok = lqr_drone.back_within_10cm > 0.0 &&
                    std::sqrt(lqr_drone.sum_sq_error / lqr_drone.n_errors) < 0.5;
    return ok ? 0 : 1;
}

} // namespace quadrotor_lqr

int main(int argc, char **argv)
{
    quadrotor_lqr::Options options;
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "--render")
        {
            options.render = true;
        }
        else if (arg == "--record" && i + 1 < argc)
        {
            options.record = argv[++i];
        }
        else
        {
            std::cerr << "Usage: quadrotor_lqr [--render] [--record DIR]\n";
            return 1;
        }
    }
    return quadrotor_lqr::run(options);
}

// A robot tows a trailer on a stiff tow bar. The robot and its speed controller are one
// subsystem, the trailer and the bar another, each a participant of its own: a thread, or a
// process that meets the other on an MQTT broker. Every macro step they exchange the motion of
// the hitch and the force of the bar, and repeat the step until the two agree.
//
// Usage: distributed_towing [--macro-step H] [--csv towing.csv]
//        distributed_towing --transport mqtt --participant robot|trailer [--broker URL]
//            [--macro-step H] [--loose]
//
// Without --transport, the example co-simulates in one process, loosely and tightly, against the
// whole system with one solver, and then with the two participants as threads on an in-process
// bus. With --transport mqtt it is one participant; start both:
//     ./distributed_towing --transport mqtt --participant robot &
//     ./distributed_towing --transport mqtt --participant trailer; wait
#include "bitsim/bitsim_config.h"
#include "bitsim/bitsim_types.h"
#include "bitsim/cosim/co_simulation.h"
#include "bitsim/cosim/cosim_participant.h"
#include "bitsim/cosim/cosim_subsystem.h"
#include "bitsim/distributed/transport.h"
#ifdef BITSIM_MQTT
#include "bitsim/distributed/mqtt_transport.h"
#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace
{
using bitsim::real_t;
using bitsim::uint_t;
using namespace bitsim::cosim;
using vec = std::vector<real_t>;

const real_t M_ROBOT = 20.0;                        // [kg]
const real_t M_TRAILER = 50.0;                      // [kg]
const real_t ROLLING = 5.0;                         // the rolling resistance of the trailer [N s/m]
const real_t K_BAR = 20000.0;                       // the stiffness of the tow bar [N/m]
const real_t D_BAR = 100.0;                         // its damping [N s/m]
const real_t L_BAR = 0.5;                           // its length [m]
const real_t KP = 400.0, KI = 200.0, F_MAX = 150.0; // the speed controller of the robot
const real_t DURATION = 8.0;                        // [s]
const uint_t SUBSTEPS = 10;

// the speed the robot is told to drive at: up to 1 m/s, along, and to a stop
real_t target_speed(real_t t)
{
    if (t < 1.0)
        return 0.0;
    if (t < 2.0)
        return t - 1.0;
    if (t < 5.0)
        return 1.0;
    if (t < 6.0)
        return 6.0 - t;
    return 0.0;
}

real_t motor_force(real_t t, real_t v, real_t integral)
{
    const real_t e = target_speed(t) - v;
    return std::clamp(KP * e + KI * integral, -F_MAX, F_MAX);
}

// the force of the bar on the robot, from the robot's hitch (x, v) and the trailer's
real_t bar_force_on_robot(const vec &robot, const vec &trailer)
{
    const real_t stretch = robot[0] - trailer[0] - L_BAR;
    return -K_BAR * stretch - D_BAR * (robot[1] - trailer[1]);
}

// the robot: state (x, v, integral of the speed error); input the force of the bar; outputs the
// hitch (x, v)
std::unique_ptr<OdeSubsystem> robot()
{
    return std::make_unique<OdeSubsystem>(
        vec{0.0, 0.0, 0.0}, 1, 2,
        [](real_t t, const vec &x, const vec &u)
        {
            const real_t f = motor_force(t, x[1], x[2]);
            return vec{x[1], (f + u[0]) / M_ROBOT, target_speed(t) - x[1]};
        },
        [](const vec &x, const vec &) {
            return vec{x[0], x[1]};
        },
        SUBSTEPS);
}

// the trailer and the bar: state (x, v) of the trailer; inputs the robot's hitch (x, v); output
// the force of the bar on the robot
std::unique_ptr<OdeSubsystem> trailer()
{
    return std::make_unique<OdeSubsystem>(
        vec{-L_BAR, 0.0}, 2, 1,
        [](real_t, const vec &x, const vec &u) {
            return vec{x[1], (-bar_force_on_robot(u, x) - ROLLING * x[1]) / M_TRAILER};
        },
        [](const vec &x, const vec &u) { return vec{bar_force_on_robot(u, x)}; }, SUBSTEPS);
}

CouplingLayout layout()
{
    CouplingLayout l;
    l.add("robot", 1, 2);
    l.add("trailer", 2, 1);
    l.connect("robot", 0, "trailer", 0);
    l.connect("robot", 1, "trailer", 1);
    l.connect("trailer", 0, "robot", 0);
    return l;
}

CouplingConfig loose() { return CouplingConfig(); }

CouplingConfig strong()
{
    CouplingConfig c;
    c.max_iterations = 30;
    c.tolerance = 1e-9;
    c.extrapolate = true; // a better first guess, fewer tries
    return c;
}

// the bar force at every macro step
struct Run
{
    vec force;
    vec robot_v;
    vec trailer_v;
    uint_t tries{0};
    uint_t not_converged{0};
    double seconds{0.0};
};

Run in_one_process(const CouplingConfig &config, real_t H)
{
    auto r = robot();
    auto t = trailer();
    CoSimulation cs(config);
    cs.add("robot", *r);
    cs.add("trailer", *t);
    const auto wiring = layout();
    for (const auto &c : wiring.connections())
    {
        cs.connect(c.from, c.output, c.to, c.input);
    }
    cs.initialise();
    Run run;
    const auto n = static_cast<uint_t>(std::llround(DURATION / H));
    for (uint_t k = 0; k < n; ++k)
    {
        cs.step(H);
        run.force.push_back(cs.inputs("robot")[0]);
        run.robot_v.push_back(r->state()[1]);
        run.trailer_v.push_back(t->state()[1]);
    }
    run.tries = cs.total_iterations();
    run.not_converged = cs.n_not_converged();
    return run;
}

// the whole system with one solver: (x_r, v_r, integral, x_t, v_t)
vec reference_force(real_t H)
{
    const auto substeps = static_cast<uint_t>(std::llround(H / 1e-5));
    OdeSubsystem whole(
        {0.0, 0.0, 0.0, -L_BAR, 0.0}, 0, 1,
        [](real_t t, const vec &x, const vec &)
        {
            const real_t bar = bar_force_on_robot({x[0], x[1]}, {x[3], x[4]});
            const real_t f = motor_force(t, x[1], x[2]);
            return vec{x[1], (f + bar) / M_ROBOT, target_speed(t) - x[1], x[4],
                       (-bar - ROLLING * x[4]) / M_TRAILER};
        },
        [](const vec &x, const vec &) {
            return vec{bar_force_on_robot({x[0], x[1]}, {x[3], x[4]})};
        },
        substeps);
    vec force;
    const auto n = static_cast<uint_t>(std::llround(DURATION / H));
    for (uint_t k = 0; k < n; ++k)
    {
        whole.do_step(k * H, H, {}, {});
        force.push_back(whole.outputs({})[0]);
    }
    return force;
}

// the value of v furthest from 0, with its sign
real_t largest(const vec &v)
{
    return *std::max_element(v.begin(), v.end(),
                             [](real_t a, real_t b) { return std::abs(a) < std::abs(b); });
}

real_t max_error(const vec &a, const vec &b)
{
    real_t e = 0.0;
    for (std::size_t k = 0; k < a.size(); ++k)
    {
        e = std::max(e, std::abs(a[k] - b[k]));
    }
    return e;
}

// one participant of a distributed run; returns the bar force at every step as it saw it
Run participate(const std::string &name, bitsim::distributed::Transport &transport,
                const CouplingConfig &config, real_t H, std::unique_ptr<OdeSubsystem> &mine)
{
    mine = name == "robot" ? robot() : trailer();
    CoSimParticipant p(layout(), name, *mine, transport, config);
    Run run;
    const auto start = std::chrono::steady_clock::now();
    p.initialise();
    const auto n = static_cast<uint_t>(std::llround(DURATION / H));
    for (uint_t k = 0; k < n; ++k)
    {
        p.step(H);
        // every participant knows the inputs of all: the force of the bar on the robot
        run.force.push_back(p.inputs("robot")[0]);
    }
    run.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    run.tries = p.total_iterations();
    run.not_converged = p.n_not_converged();
    p.leave();
    return run;
}

} // namespace

int main(int argc, char **argv)
{
    real_t H = 0.01;
    std::string csv_path = "towing.csv", transport = "inprocess", participant,
                broker = "tcp://localhost:1883";
    bool use_loose = false;
    for (int i = 1; i < argc; ++i)
    {
        const std::string a = argv[i];
        auto next = [&]() -> std::string
        {
            if (i + 1 >= argc)
            {
                std::cerr << a << " needs a value\n";
                std::exit(1);
            }
            return argv[++i];
        };
        if (a == "--macro-step")
            H = std::stod(next());
        else if (a == "--csv")
            csv_path = next();
        else if (a == "--transport")
            transport = next();
        else if (a == "--participant")
            participant = next();
        else if (a == "--broker")
            broker = next();
        else if (a == "--loose")
            use_loose = true;
        else
        {
            std::cerr << "unknown option " << a << "\n";
            return 1;
        }
    }

    if (transport == "mqtt")
    {
#ifdef BITSIM_MQTT
        if (participant != "robot" && participant != "trailer")
        {
            std::cerr << "--transport mqtt needs --participant robot or trailer\n";
            return 1;
        }
        bitsim::distributed::MQTT_Transport mqtt(broker, "distributed_towing");
        std::unique_ptr<OdeSubsystem> mine;
        const auto run = participate(participant, mqtt, use_loose ? loose() : strong(), H, mine);
        const real_t peak = largest(run.force);
        std::printf("%s: %zu macro steps, %u tries, in %.2f s, %.0f microseconds a try; the "
                    "largest bar force %.9f N\n",
                    participant.c_str(), run.force.size(), static_cast<unsigned>(run.tries),
                    run.seconds, 1e6 * run.seconds / run.tries, peak);
        return 0;
#else
        std::cerr << "this bitsim was built without MQTT\n";
        return 1;
#endif
    }
    if (transport != "inprocess")
    {
        std::cerr << "--transport is inprocess or mqtt\n";
        return 1;
    }

    std::printf("A robot tows a trailer on a stiff bar for %.0f s, macro step %.3f s\n\n", DURATION,
                H);
    const auto ref = reference_force(H);
    const auto l = in_one_process(loose(), H);
    const auto s = in_one_process(strong(), H);
    const auto n = ref.size();
    auto peak = [](const vec &v)
    {
        real_t p = 0.0;
        for (const real_t f : v)
            p = std::max(p, std::abs(f));
        return p;
    };
    std::printf("1. In one process: the force of the bar [N]\n");
    std::printf("                          largest force   largest error   tries per step\n");
    std::printf("   whole system, one solver %12.1f\n", peak(ref));
    std::printf("   loose coupling           %12.4g    %12.4g   %14.1f\n", peak(l.force),
                max_error(l.force, ref), static_cast<real_t>(l.tries) / n);
    std::printf("   strong coupling          %12.1f    %12.4g   %14.1f\n", peak(s.force),
                max_error(s.force, ref), static_cast<real_t>(s.tries) / n);
    std::printf("   strong steps that did not converge: %u\n\n",
                static_cast<unsigned>(s.not_converged));

    std::printf("2. The robot and the trailer in participants of their own, threads on a bus\n");
    bitsim::distributed::InProcessBus bus;
    auto tr = bus.connect();
    auto tt = bus.connect();
    Run on_robot, on_trailer;
    std::unique_ptr<OdeSubsystem> robot_sub, trailer_sub;
    std::thread a([&] { on_robot = participate("robot", *tr, strong(), H, robot_sub); });
    std::thread b([&] { on_trailer = participate("trailer", *tt, strong(), H, trailer_sub); });
    a.join();
    b.join();
    std::printf("   %u tries in %.3f s: %.0f microseconds a try for the exchange and the step\n",
                static_cast<unsigned>(on_robot.tries), on_robot.seconds,
                1e6 * on_robot.seconds / on_robot.tries);
    std::printf("   the largest bar force %.9f N\n", largest(on_robot.force));
    std::printf("   both saw the bar force of the one process at every step, to the last bit: "
                "%s\n",
                on_robot.force == s.force && on_trailer.force == s.force ? "true" : "false");

    std::ofstream csv(csv_path);
    csv << "t,target,robot,trailer,force whole system,force strong,force loose\n";
    for (std::size_t k = 0; k < n; ++k)
    {
        const real_t t = (k + 1) * H;
        csv << t << "," << target_speed(t) << "," << s.robot_v[k] << "," << s.trailer_v[k] << ","
            << ref[k] << "," << s.force[k] << "," << l.force[k] << "\n";
    }
    std::printf("\nthe speeds and the bar force are in %s\n", csv_path.c_str());
    return 0;
}

// A simulated robot follows a real one, in wall-clock mode.
//
// The "real" robot drives a circle in real time and sends its measured pose ten times a second,
// as the code on a Raspberry Pi would: a StateMessage stamped with the wall-clock time. Its
// messages reach the simulation over a network that delays them by 80 ms, give or take 20 ms,
// and loses one in ten, and for 1.5 s in the middle they stop altogether, as when the Wi-Fi drops.
// The simulation runs at 50 steps a second in wall-clock mode and drives a simulated robot that
// keeps 0.5 m behind the real one. It does not wait for the real robot, which could not wait for
// it; it uses the newest pose it has, moved forward by its age, and stops its robot when the pose
// is too old to trust.
//
// Usage: hardware_in_the_loop_sync [--duration S]
//        hardware_in_the_loop_sync --transport mqtt --role robot|sim [--duration S] [--broker URL]
//
// In-process (the default) the real robot is a thread, the example runs twice, with and without
// moving the pose forward, and compares what the simulation believed with where the robot was.
// With --transport mqtt the two sides are separate processes, and --role robot is the side a real
// robot replaces. Writes hil_extrapolated.csv and hil_newest.csv.

#include "bitsim/bitsim_config.h"
#include "bitsim/bitsim_consts.h"
#include "bitsim/bitsim_types.h"
#include "bitsim/distributed/state_message.h"
#include "bitsim/distributed/synchroniser.h"
#include "bitsim/distributed/transport.h"
#include "bitsim/utils/io/print_utils.h"

#ifdef BITSIM_MQTT
#include "bitsim/distributed/mqtt_transport.h"
#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <thread>

namespace hardware_in_the_loop_sync
{

using bitsim::real_t;
using bitsim::uint_t;
using namespace bitsim::distributed;
using steady = std::chrono::steady_clock;

const real_t PI = bitsim::consts::maths::PI;
const real_t SPEED = 0.4;         // [m/s], the real robot
const real_t TURN = 0.4;          // [rad/s], so a circle of 1 m
const real_t REPORT_PERIOD = 0.1; // the real robot reports ten times a second
const real_t DROP_FROM = 0.45;    // the reports stop for DROP_LENGTH from this share of the run
const real_t DROP_LENGTH = 1.5;   // [s]
const real_t SIM_PERIOD = 0.02;   // the simulation steps 50 times a second
const real_t BEHIND = 0.5;        // [m], how far the simulated robot keeps behind
const real_t STALE_AFTER = 0.4;   // [s]

struct Pose
{
    real_t x, y, theta;
};

// Where the real robot truly is, t seconds after it started
Pose true_pose(real_t t)
{
    const real_t theta = TURN * t;
    const real_t r = SPEED / TURN;
    return {r * std::sin(theta), r * (1.0 - std::cos(theta)), theta};
}

// The real robot: drive, and every REPORT_PERIOD send the measured pose, except while the
// network is down. The pose is measured with 1 cm of noise
void real_robot(Transport &transport, real_t duration, steady::time_point start)
{
    std::mt19937 rng(3);
    std::normal_distribution<real_t> noise(0.0, 0.01);
    const real_t drop_from = DROP_FROM * duration;
    for (uint_t k = 0;; ++k)
    {
        const real_t t = k * REPORT_PERIOD;
        if (t > duration)
        {
            break;
        }
        std::this_thread::sleep_until(
            start + std::chrono::duration_cast<steady::duration>(std::chrono::duration<real_t>(t)));
        if (t >= drop_from && t < drop_from + DROP_LENGTH)
        {
            continue; // the Wi-Fi is down
        }
        const Pose p = true_pose(t);
        StateMessage m;
        m.participant = "robot";
        m.step = k;
        m.time = t;
        m.stamp = wall_time_now();
        m.bodies.push_back(BodyState::planar("chassis", p.x + noise(rng), p.y + noise(rng), p.theta,
                                             SPEED * std::cos(p.theta), SPEED * std::sin(p.theta),
                                             TURN, {0.4, 0.3, 0.1}));
        transport.publish(m.encode());
    }
}

struct Statistics
{
    real_t mean_error{0.0}, max_error{0.0}, mean_age{0.0};
    uint_t n_steps{0}, n_stale{0}, n_stopped{0}, n_overruns{0};
    real_t closest{1e9}, worst_gap_error{0.0};
};

// The simulation: a robot that keeps BEHIND behind the real one, with what the synchroniser
// says of it. With a known start, the true pose is known too, to measure the error
Statistics simulation(Transport &transport, real_t duration, bool extrapolate,
                      steady::time_point start, const std::string &csv)
{
    SynchroniserConfig config;
    config.mode = SyncMode::WALL_CLOCK;
    config.sync_period = SIM_PERIOD;
    config.stale_after = STALE_AFTER;
    config.extrapolate = extrapolate;
    config.peers = {"robot"};
    Synchroniser sync(transport, "sim", config);

    std::ofstream out(csv);
    out << "t,true_x,true_y,believed_x,believed_y,age,stale,sim_x,sim_y\n";
    Statistics s;
    Pose me{-BEHIND, 0.0, 0.0};
    real_t v = 0.0, w = 0.0;
    const auto n = static_cast<uint_t>(duration / SIM_PERIOD);
    for (uint_t k = 0; k < n; ++k)
    {
        const auto &peers = sync.exchange(
            k, {BodyState::planar("chassis", me.x, me.y, me.theta, v * std::cos(me.theta),
                                  v * std::sin(me.theta), w, {0.4, 0.3, 0.1})});
        // the time since the real robot started, to look up where it truly is
        const real_t t = std::chrono::duration<real_t>(steady::now() - start).count();
        const auto &robot = peers.front();
        ++s.n_steps;

        if (robot.stale || robot.bodies.empty())
        {
            // too old to trust: stop
            ++s.n_stale;
            ++s.n_stopped;
            v = 0.0;
            w = 0.0;
        }
        else
        {
            const auto &b = robot.bodies.front();
            const Pose truth = true_pose(t);
            const real_t error = std::hypot(b.position[0] - truth.x, b.position[1] - truth.y);
            s.mean_error += error;
            s.max_error = std::max(s.max_error, error);
            s.mean_age += robot.age;
            out << t << "," << truth.x << "," << truth.y << "," << b.position[0] << ","
                << b.position[1] << "," << robot.age << ",0," << me.x << "," << me.y << "\n";

            // head for the point BEHIND behind where we believe the robot is
            const real_t yaw = b.yaw();
            const real_t gx = b.position[0] - BEHIND * std::cos(yaw);
            const real_t gy = b.position[1] - BEHIND * std::sin(yaw);
            const real_t d = std::hypot(gx - me.x, gy - me.y);
            const real_t heading_error =
                std::remainder(std::atan2(gy - me.y, gx - me.x) - me.theta, 2 * PI);
            v = std::clamp(1.5 * d, 0.0, 0.8) * std::max(0.0, std::cos(heading_error));
            w = std::clamp(4.0 * heading_error, -3.0, 3.0);
        }
        me.x += v * std::cos(me.theta) * SIM_PERIOD;
        me.y += v * std::sin(me.theta) * SIM_PERIOD;
        me.theta += w * SIM_PERIOD;

        const Pose truth = true_pose(t);
        s.closest = std::min(s.closest, std::hypot(truth.x - me.x, truth.y - me.y));
    }
    sync.leave();
    const uint_t n_fresh = s.n_steps - s.n_stale;
    if (n_fresh)
    {
        s.mean_error /= n_fresh;
        s.mean_age /= n_fresh;
    }
    s.n_overruns = sync.n_overruns();
    return s;
}

void report(const std::string &name, const Statistics &s)
{
    std::cout << std::fixed << std::setprecision(3) << "  " << name << ": error of the believed "
              << "pose " << s.mean_error << " m on average, at most " << s.max_error
              << " m; mean age " << s.mean_age << " s\n"
              << "    " << s.n_stale << " of " << s.n_steps
              << " steps stale, the robot stopped for " << std::setprecision(2)
              << s.n_stopped * SIM_PERIOD << " s; closest to the real robot " << s.closest << " m; "
              << s.n_overruns << " steps started late\n";
}

Statistics run_in_process(real_t duration, bool extrapolate, const std::string &csv)
{
    InProcessBus bus;
    auto robot_side = bus.connect();
    LossyTransport::Config network;
    network.latency = 0.08;
    network.jitter = 0.02;
    network.loss = 0.1;
    LossyTransport sim_side(bus.connect(), network);

    const auto start = steady::now();
    std::thread robot([&] { real_robot(*robot_side, duration, start); });
    const auto s = simulation(sim_side, duration, extrapolate, start, csv);
    robot.join();
    return s;
}

int run(real_t duration, const std::string &transport, const std::string &role,
        const std::string &broker)
{
    if (transport == "mqtt")
    {
#ifdef BITSIM_MQTT
        MQTT_Transport t(broker, "hardware_in_the_loop_sync");
        const auto start = steady::now();
        if (role == "robot")
        {
            std::cout << "  sending the robot's pose for " << duration << " s\n";
            real_robot(t, duration, start);
            return 0;
        }
        if (role == "sim")
        {
            // the start of the two processes differs, so the error against the true pose is off
            // by that; the ages and the stale steps are not
            report("over MQTT", simulation(t, duration, true, start, "hil_mqtt.csv"));
            return 0;
        }
        std::cerr << "--role must be robot or sim\n";
        return 1;
#else
        std::cerr << "bitsim was built without MQTT\n";
        return 1;
#endif
    }

    bitsim::utils::io::print_title("1. Moving the pose forward by its age");
    report("extrapolated", run_in_process(duration, true, "hil_extrapolated.csv"));
    std::cout << "\n";
    bitsim::utils::io::print_title("2. Using the newest pose as it is");
    report("newest", run_in_process(duration, false, "hil_newest.csv"));
    return 0;
}

} // namespace hardware_in_the_loop_sync

int main(int argc, char **argv)
{
    bitsim::real_t duration = 8.0;
    std::string transport = "inprocess", role, broker = "tcp://localhost:1883";
    for (int i = 1; i + 1 < argc; i += 2)
    {
        const std::string flag = argv[i], value = argv[i + 1];
        if (flag == "--duration")
            duration = std::strtod(value.c_str(), nullptr);
        else if (flag == "--transport")
            transport = value;
        else if (flag == "--role")
            role = value;
        else if (flag == "--broker")
            broker = value;
        else
        {
            std::cerr << "unknown option " << flag << "\n";
            return 1;
        }
    }
    return hardware_in_the_loop_sync::run(duration, transport, role, broker);
}

// Differential drive robots cross a circle, each simulated by its own participant: a thread of
// this process, or a process of its own, on this machine or another. Every 0.05 s of simulated
// time the participants exchange where their robots are, through bitsim's Synchroniser, and each
// robot steers around the others it hears of.
//
// Usage: distributed_robots [--robots N] [--mode lockstep|wallclock] [--speed S]
//                           [--latency S] [--loss P]
//                           [--transport inprocess|mqtt] [--participant K] [--broker URL]
//                           [--resend S]
//
// With the in-process transport (the default) every robot is a thread of this process, and the
// example runs the crossing three times: in lockstep, in lockstep over a network that delays and
// loses messages, and in wall-clock mode over the same network. With --transport mqtt it simulates
// robot K only, and the others are other processes on the same broker:
//
//     for k in 0 1 2 3; do ./distributed_robots --transport mqtt --participant $k & done; wait
//
// Writes robot_K.csv, the path of robot K in lockstep, to the working directory.

#include "bitsim/bitsim_config.h"
#include "bitsim/bitsim_consts.h"
#include "bitsim/bitsim_types.h"
#include "bitsim/distributed/state_message.h"
#include "bitsim/distributed/synchroniser.h"
#include "bitsim/distributed/transport.h"
#include "bitsim/dynamics/diff_drive_dynamics.h"
#include "bitsim/dynamics/system_state.h"
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
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace distributed_robots
{

using bitsim::real_t;
using bitsim::uint_t;
using namespace bitsim::distributed;

const real_t PI = bitsim::consts::maths::PI;
const real_t CIRCLE = 2.0;       // [m], the radius of the circle the robots start on
const real_t RADIUS = 0.15;      // [m], the robots are discs this big
const real_t MAX_V = 0.5;        // [m/s]
const real_t MAX_W = 2.0;        // [rad/s]
const real_t DT = 0.01;          // [s], the step of the dynamics
const uint_t STEPS_PER_SYNC = 5; // so the robots exchange states every 0.05 s
const real_t TIME_LIMIT = 40.0;  // [s]

struct Options
{
    uint_t n_robots{4};
    SyncMode mode{SyncMode::LOCKSTEP};
    real_t speed{1.0}; // wall-clock: simulated seconds per second
    LossyTransport::Config network{};
    std::string transport{"inprocess"};
    int participant{-1};
    std::string broker{"tcp://localhost:1883"};
    real_t resend{0.02}; // lockstep: send the last steps again this often while waiting [s]
    bool write_paths{true};
};

std::string name_of(uint_t k) { return "robot_" + std::to_string(k); }

real_t wrap(real_t a) { return std::remainder(a, 2.0 * PI); }

// Head for the goal, pushed away from, and to the right of, the robots closer than 1 m
std::array<real_t, 2> steer(const bitsim::dynamics::SysState<3> &s,
                            const std::array<real_t, 2> &goal, const std::vector<PeerState> &peers)
{
    const real_t x = s[0], y = s[1], theta = s[2];
    const real_t distance = std::hypot(goal[0] - x, goal[1] - y);
    real_t dx = (goal[0] - x) / std::max(distance, 1e-9);
    real_t dy = (goal[1] - y) / std::max(distance, 1e-9);
    real_t slow = 1.0;
    for (const auto &peer : peers)
    {
        for (const auto &b : peer.bodies)
        {
            const real_t rx = b.position[0] - x, ry = b.position[1] - y;
            const real_t r = std::hypot(rx, ry);
            if (r > 1.0 || r < 1e-9)
            {
                continue;
            }
            const real_t push = 0.3 * (1.0 / std::max(r - 2.0 * RADIUS, 0.05) - 1.0);
            dx += push * (ry - rx) / r;
            dy -= push * (rx + ry) / r;
            if ((rx * std::cos(theta) + ry * std::sin(theta)) / r > 0.0)
            {
                slow = std::min(slow, std::clamp((r - 2.0 * RADIUS) / 0.5, 0.2, 1.0));
            }
        }
    }
    const real_t error = wrap(std::atan2(dy, dx) - theta);
    return {std::min(MAX_V, distance) * std::max(0.0, std::cos(error)) * slow,
            std::clamp(3.0 * error, -MAX_W, MAX_W)};
}

struct Result
{
    std::string name;
    bool arrived{false};
    real_t time{0.0};
    std::vector<std::array<real_t, 3>> path; // t, x, y at every exchange
    uint_t n_resent{0}, n_overruns{0};
    real_t max_wait{0.0};
};

// Simulate robot k until it arrives, exchanging its state every STEPS_PER_SYNC steps
Result simulate(uint_t k, const Options &options, Transport &transport)
{
    const real_t angle = 2.0 * PI * k / options.n_robots;
    bitsim::dynamics::SysState<3> state({std::make_pair("X", CIRCLE * std::cos(angle)),
                                         std::make_pair("Y", CIRCLE * std::sin(angle)),
                                         std::make_pair("Theta", wrap(angle + PI))});
    const std::array<real_t, 2> goal{-CIRCLE * std::cos(angle), -CIRCLE * std::sin(angle)};

    SynchroniserConfig config;
    config.mode = options.mode;
    config.sync_period = STEPS_PER_SYNC * DT;
    config.real_time_factor = options.speed;
    config.timeout = 30.0;
    // a lost message costs a resend period of waiting
    config.resend_period = options.resend;
    config.stale_after = 0.5;
    for (uint_t j = 0; j < options.n_robots; ++j)
    {
        if (j != k)
        {
            config.peers.push_back(name_of(j));
        }
    }
    Synchroniser sync(transport, name_of(k), config);

    Result result;
    result.name = name_of(k);
    std::array<real_t, 2> command{0.0, 0.0};
    const auto n_syncs = static_cast<uint_t>(TIME_LIMIT / config.sync_period);
    for (uint_t step = 0; step < n_syncs; ++step)
    {
        const real_t t = sync.time_of(step);
        const real_t vx = command[0] * std::cos(state[2]), vy = command[0] * std::sin(state[2]);
        const auto &peers =
            sync.exchange(step, {BodyState::planar("chassis", state[0], state[1], state[2], vx, vy,
                                                   command[1], {2 * RADIUS, 2 * RADIUS, 0.1})});
        result.path.push_back({t, state[0], state[1]});
        if (std::hypot(goal[0] - state[0], goal[1] - state[1]) < 0.1)
        {
            result.arrived = true;
            result.time = t;
            break;
        }
        // the controller sees the others as they were at this exchange, until the next one
        command = steer(state, goal, peers);
        for (uint_t i = 0; i < STEPS_PER_SYNC; ++i)
        {
            state = bitsim::dynamics::DiffDriveDynamics::integrate_state_v2(state, DT, command[0],
                                                                            command[1], {0.0, 0.0});
        }
    }
    // an arrived robot stays where it is: the others still see its last state
    sync.leave();
    result.n_resent = sync.n_resent();
    result.n_overruns = sync.n_overruns();
    result.max_wait = sync.max_wait();

    if (options.write_paths)
    {
        std::ofstream out(result.name + ".csv");
        out << std::setprecision(17) << "t,x,y\n";
        for (const auto &p : result.path)
        {
            out << p[0] << "," << p[1] << "," << p[2] << "\n";
        }
    }
    return result;
}

// The closest two robots came, from their paths at the exchanges
real_t closest(const std::vector<Result> &results)
{
    real_t best = 1e9;
    for (std::size_t a = 0; a < results.size(); ++a)
    {
        for (std::size_t b = a + 1; b < results.size(); ++b)
        {
            const auto &pa = results[a].path, &pb = results[b].path;
            for (std::size_t i = 0; i < std::max(pa.size(), pb.size()); ++i)
            {
                const auto &qa = pa[std::min(i, pa.size() - 1)];
                const auto &qb = pb[std::min(i, pb.size() - 1)];
                best = std::min(best, std::hypot(qa[1] - qb[1], qa[2] - qb[2]));
            }
        }
    }
    return best;
}

void report(const std::vector<Result> &results, real_t wall_seconds, SyncMode mode)
{
    real_t last = 0.0;
    uint_t arrived = 0, resent = 0, overruns = 0;
    real_t max_wait = 0.0;
    for (const auto &r : results)
    {
        arrived += r.arrived ? 1 : 0;
        last = std::max(last, r.time);
        resent += r.n_resent;
        overruns += r.n_overruns;
        max_wait = std::max(max_wait, r.max_wait);
    }
    std::cout << std::fixed << std::setprecision(2) << "  " << arrived << " of " << results.size()
              << " robots arrived, the last after " << last << " s of simulated time, in "
              << wall_seconds << " s; closest centres " << closest(results) << " m\n"
              << std::setprecision(3);
    if (mode == SyncMode::LOCKSTEP)
    {
        std::cout << "  " << resent << " times a robot sent its last steps again; the longest "
                  << "wait for the others " << max_wait << " s\n";
    }
    else
    {
        std::cout << "  " << overruns << " steps started more than a period late; the latest "
                  << max_wait << " s\n";
    }
}

// Every robot a thread of this process
std::vector<Result> run_in_process(const Options &options)
{
    InProcessBus bus;
    std::vector<std::unique_ptr<Transport>> transports;
    for (uint_t k = 0; k < options.n_robots; ++k)
    {
        auto t = bus.connect();
        if (options.network.latency > 0.0 || options.network.loss > 0.0)
        {
            auto network = options.network;
            network.seed += k;
            t = std::make_unique<LossyTransport>(std::move(t), network);
        }
        transports.push_back(std::move(t));
    }
    std::vector<Result> results(options.n_robots);
    std::vector<std::thread> threads;
    for (uint_t k = 0; k < options.n_robots; ++k)
    {
        threads.emplace_back([&, k] { results[k] = simulate(k, options, *transports[k]); });
    }
    for (auto &t : threads)
    {
        t.join();
    }
    return results;
}

template <typename F> real_t timed(F &&f)
{
    const auto start = std::chrono::steady_clock::now();
    f();
    return std::chrono::duration<real_t>(std::chrono::steady_clock::now() - start).count();
}

int run(const Options &options)
{
    if (options.transport == "mqtt")
    {
#ifdef BITSIM_MQTT
        if (options.participant < 0 || static_cast<uint_t>(options.participant) >= options.n_robots)
        {
            std::cerr << "--transport mqtt needs --participant K, 0 <= K < " << options.n_robots
                      << "\n";
            return 1;
        }
        MQTT_Transport transport(options.broker, "distributed_robots");
        Result result;
        const real_t wall =
            timed([&] { result = simulate(options.participant, options, transport); });
        std::cout << result.name << (result.arrived ? " arrived after " : " gave up after ")
                  << std::fixed << std::setprecision(2) << result.time << " s, in " << wall
                  << " s; " << result.n_resent << " resends\n";
        return 0;
#else
        std::cerr << "bitsim was built without MQTT\n";
        return 1;
#endif
    }

    if (options.participant >= 0)
    {
        std::cerr << "--participant is for --transport mqtt\n";
        return 1;
    }

    bitsim::utils::io::print_title("1. In lockstep");
    Options lockstep = options;
    lockstep.mode = SyncMode::LOCKSTEP;
    lockstep.network = {};
    std::vector<Result> first, again;
    real_t wall = timed([&] { first = run_in_process(lockstep); });
    report(first, wall, SyncMode::LOCKSTEP);
    lockstep.write_paths = false;
    again = run_in_process(lockstep);
    bool same = true;
    for (uint_t k = 0; k < first.size(); ++k)
    {
        same = same && first[k].path == again[k].path;
    }
    std::cout << "  a second run gives the same paths: " << std::boolalpha << same << "\n\n";

    bitsim::utils::io::print_title("2. In lockstep, over a network that delays and loses");
    Options lossy = lockstep;
    lossy.network = options.network;
    if (lossy.network.latency == 0.0 && lossy.network.loss == 0.0)
    {
        lossy.network.latency = 0.005;
        lossy.network.jitter = 0.002;
        lossy.network.loss = 0.1;
    }
    std::vector<Result> over_network;
    wall = timed([&] { over_network = run_in_process(lossy); });
    report(over_network, wall, SyncMode::LOCKSTEP);
    same = true;
    for (uint_t k = 0; k < first.size(); ++k)
    {
        same = same && first[k].path == over_network[k].path;
    }
    std::cout << "  " << std::setprecision(0) << 100 * lossy.network.loss << " % of the messages "
              << "lost, " << 1000 * lossy.network.latency << " ms of latency; the paths are "
              << (same ? "the same as in 1" : "NOT the same as in 1") << "\n\n";

    bitsim::utils::io::print_title("3. In wall-clock mode, over the same network");
    Options wall_clock = lossy;
    wall_clock.mode = SyncMode::WALL_CLOCK;
    wall_clock.speed = options.speed;
    std::vector<Result> real_time;
    wall = timed([&] { real_time = run_in_process(wall_clock); });
    report(real_time, wall, SyncMode::WALL_CLOCK);
    std::cout << "  at " << std::setprecision(1) << wall_clock.speed
              << " times real time; the paths differ from run to run\n";
    return 0;
}

} // namespace distributed_robots

int main(int argc, char **argv)
{
    distributed_robots::Options options;
    options.speed = 4.0;
    for (int i = 1; i + 1 < argc; i += 2)
    {
        const std::string flag = argv[i], value = argv[i + 1];
        if (flag == "--robots")
            options.n_robots = std::strtoul(value.c_str(), nullptr, 10);
        else if (flag == "--mode")
            options.mode = value == "wallclock" ? bitsim::distributed::SyncMode::WALL_CLOCK
                                                : bitsim::distributed::SyncMode::LOCKSTEP;
        else if (flag == "--speed")
            options.speed = std::strtod(value.c_str(), nullptr);
        else if (flag == "--latency")
            options.network.latency = std::strtod(value.c_str(), nullptr);
        else if (flag == "--loss")
            options.network.loss = std::strtod(value.c_str(), nullptr);
        else if (flag == "--transport")
            options.transport = value;
        else if (flag == "--participant")
            options.participant = std::atoi(value.c_str());
        else if (flag == "--broker")
            options.broker = value;
        else if (flag == "--resend")
            options.resend = std::strtod(value.c_str(), nullptr);
        else
        {
            std::cerr << "unknown option " << flag << "\n";
            return 1;
        }
    }
    return distributed_robots::run(options);
}

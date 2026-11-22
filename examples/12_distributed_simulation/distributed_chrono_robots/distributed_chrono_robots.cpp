// Two of bitsim's Chrono robots drive towards each other, each simulated by its own participant,
// in a Chrono system of its own. A participant sees the other robot only as a ghost, a box that
// CHRONO_Ghosts moves to where the other participant says its chassis is; the front sonar of the
// robot hears the ghost like any other body, and the robot stops in front of it.
//
// Usage: distributed_chrono_robots [--no-ghosts] [--mode lockstep|wallclock]
//                                  [--transport inprocess|mqtt] [--participant K] [--broker URL]
//
// With the in-process transport (the default) both robots are threads of this process. With
// --transport mqtt it simulates robot K only, and the other is another process on the broker:
//
//     ./distributed_chrono_robots --transport mqtt --participant 0 &
//     ./distributed_chrono_robots --transport mqtt --participant 1
//
// --no-ghosts leaves the ghosts out, to show what they are for. Writes chrono_robot_K.csv, the
// time, the position of the front of the chassis and the sonar reading of robot K.

#include "bitsim/bitsim_config.h"
#include "bitsim/bitsim_consts.h"
#include "bitsim/bitsim_types.h"
#include "bitsim/distributed/chrono_ghosts.h"
#include "bitsim/distributed/synchroniser.h"
#include "bitsim/distributed/transport.h"
#include "bitsim/rigid_bodies/chrono_robots/diff_drive_robot.h"
#include "bitsim/utils/io/print_utils.h"

#ifdef BITSIM_MQTT
#include "bitsim/distributed/mqtt_transport.h"
#endif

#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChContactMaterialNSC.h>
#include <chrono/physics/ChSystemNSC.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace distributed_chrono_robots
{

using bitsim::real_t;
using bitsim::uint_t;
using bitsim::rb::bitsim_chrono::CHRONO_DiffDriveRobot;
using namespace bitsim::distributed;

const std::string ROBOT_FILE = bitsim::consts::ROBOTS_DIR + "/bitsim_diff_drive_robot.json";
const std::array<real_t, 3> CHASSIS{0.5, 0.3, 0.1}; // the chassis of the robot file [m]
const real_t START = 2.0;                           // the axle centres start at x = -2 and 2 [m]
const real_t SPEED = 0.3;                           // [m/s]
const real_t STOP_AT = 0.4;                         // the sonar distance to stop at [m]
const real_t STEP = 0.01;                           // the step of Chrono [s]
const uint_t STEPS_PER_SYNC = 2;                    // exchange every 0.02 s
const real_t DURATION = 12.0;                       // [s]

struct Options
{
    bool ghosts{true};
    SyncMode mode{SyncMode::LOCKSTEP};
    std::string transport{"inprocess"};
    int participant{-1};
    std::string broker{"tcp://localhost:1883"};
};

std::string name_of(uint_t k) { return "robot_" + std::to_string(k); }

struct Result
{
    real_t stopped_at{-1.0}; // the time the robot stopped, -1 if it did not
    real_t sonar{0.0};       // the last sonar reading [m]
    real_t front{0.0};       // the x of the front of the chassis at the end [m]
};

// The world of one participant: the floor, its own robot, and the ghosts of the others
class World
{
  public:
    explicit World(uint_t k, bool with_ghosts) : ghosts_(system_), with_ghosts_(with_ghosts)
    {
        system_.SetGravitationalAcceleration(chrono::ChVector3d(0, 0, -bitsim::consts::maths::G));
        system_.SetCollisionSystemType(chrono::ChCollisionSystem::Type::BULLET);
        // as in line_following_infrared_sonar, which drives the same robot
        system_.SetSolverType(chrono::ChSolver::Type::BARZILAIBORWEIN);
        system_.GetSolver()->AsIterative()->SetMaxIterations(40);
        auto material = chrono_types::make_shared<chrono::ChContactMaterialNSC>();
        material->SetFriction(0.8f);
        auto ground = chrono_types::make_shared<chrono::ChBodyEasyBox>(10.0, 4.0, 0.1, 1000.0, true,
                                                                       true, material);
        ground->SetPos(chrono::ChVector3d(0.0, 0.0, -0.05));
        ground->SetFixed(true);
        system_.AddBody(ground);

        robot_.load(std::filesystem::path(ROBOT_FILE));
        // robot 0 on the left facing right, robot 1 on the right facing left
        robot_.set_world_pose(k == 0 ? -START : START, 0.0,
                              k == 0 ? 0.0 : bitsim::consts::maths::PI);
        robot_.add_to_sys(system_);
        direction_ = k == 0 ? 1.0 : -1.0;
    }

    // What the others need to know of this robot: its chassis
    std::vector<BodyState> bodies() const
    {
        return {chrono_body_state("chassis", *robot_.get_chassis(), CHASSIS)};
    }

    void place_ghosts(const std::vector<PeerState> &peers)
    {
        if (with_ghosts_)
        {
            ghosts_.update(peers);
        }
    }

    // Drive until the sonar hears something closer than STOP_AT, slowing down before it
    void step(real_t &sonar)
    {
        auto sensor = robot_.get_sonar();
        sonar = sensor->read_values().at(0);
        const real_t v = SPEED * std::clamp((sonar - STOP_AT) / 0.5, 0.0, 1.0);
        robot_.set_velocity(v, 0.0);
        robot_.update(STEP);
        system_.DoStepDynamics(STEP);
    }

    real_t time() const { return system_.GetChTime(); }

    // the front of the chassis, along x
    real_t front() const
    {
        return robot_.get_chassis()->GetPos().x() + direction_ * 0.5 * CHASSIS[0];
    }

  private:
    chrono::ChSystemNSC system_;
    CHRONO_DiffDriveRobot robot_;
    CHRONO_Ghosts ghosts_;
    bool with_ghosts_;
    real_t direction_{1.0};
};

Result simulate(uint_t k, const Options &options, Transport &transport)
{
    World world(k, options.ghosts);

    SynchroniserConfig config;
    config.mode = options.mode;
    config.sync_period = STEPS_PER_SYNC * STEP;
    config.peers = {name_of(1 - k)};
    config.timeout = 30.0;
    config.resend_period = 0.02;
    Synchroniser sync(transport, name_of(k), config);

    Result result;
    std::ofstream out("chrono_robot_" + std::to_string(k) + ".csv");
    out << "t,front,sonar\n";
    const auto n_syncs = static_cast<uint_t>(DURATION / config.sync_period);
    real_t sonar = 0.0;
    for (uint_t s = 0; s < n_syncs; ++s)
    {
        world.place_ghosts(sync.exchange(s, world.bodies()));
        for (uint_t i = 0; i < STEPS_PER_SYNC; ++i)
        {
            world.step(sonar);
        }
        out << world.time() << "," << world.front() << "," << sonar << "\n";
        if (result.stopped_at < 0.0 && sonar <= STOP_AT + 0.005)
        {
            result.stopped_at = world.time();
        }
    }
    sync.leave();
    result.sonar = sonar;
    result.front = world.front();
    return result;
}

// results[i] is the result of robot first + i
void report(const std::vector<Result> &results, uint_t first = 0)
{
    for (uint_t i = 0; i < results.size(); ++i)
    {
        const auto &r = results[i];
        std::cout << std::fixed << std::setprecision(2) << "  " << name_of(first + i) << ": "
                  << (r.stopped_at >= 0.0
                          ? "stopped after " + std::to_string(r.stopped_at).substr(0, 5) + " s"
                          : std::string("never stopped"))
                  << ", its sonar reads " << r.sonar
                  << " m, the front of its chassis is at x = " << std::showpos << r.front
                  << std::noshowpos << " m\n";
    }
    if (results.size() == 2)
    {
        const real_t gap = results[1].front - results[0].front;
        std::cout << "  the gap between the two fronts is " << std::setprecision(2) << gap << " m"
                  << (gap < 0.0 ? ": in one world they would have crashed into each other" : "")
                  << "\n";
    }
}

std::vector<Result> run_in_process(const Options &options)
{
    InProcessBus bus;
    std::vector<std::unique_ptr<Transport>> transports;
    transports.push_back(bus.connect());
    transports.push_back(bus.connect());
    std::vector<Result> results(2);
    std::vector<std::thread> threads;
    for (uint_t k = 0; k < 2; ++k)
    {
        threads.emplace_back([&, k] { results[k] = simulate(k, options, *transports[k]); });
    }
    for (auto &t : threads)
    {
        t.join();
    }
    return results;
}

int run(const Options &options)
{
    if (options.transport == "mqtt")
    {
#ifdef BITSIM_MQTT
        if (options.participant != 0 && options.participant != 1)
        {
            std::cerr << "--transport mqtt needs --participant 0 or 1\n";
            return 1;
        }
        MQTT_Transport transport(options.broker, "distributed_chrono_robots");
        report({simulate(options.participant, options, transport)}, options.participant);
        return 0;
#else
        std::cerr << "bitsim was built without MQTT\n";
        return 1;
#endif
    }

    bitsim::utils::io::print_title(options.ghosts ? "Each robot sees the ghost of the other"
                                                  : "Without ghosts");
    report(run_in_process(options));
    return 0;
}

} // namespace distributed_chrono_robots

int main(int argc, char **argv)
{
    distributed_chrono_robots::Options options;
    for (int i = 1; i < argc; ++i)
    {
        const std::string flag = argv[i];
        if (flag == "--no-ghosts")
        {
            options.ghosts = false;
            continue;
        }
        if (i + 1 >= argc)
        {
            std::cerr << "option " << flag << " needs a value\n";
            return 1;
        }
        const std::string value = argv[++i];
        if (flag == "--mode")
            options.mode = value == "wallclock" ? bitsim::distributed::SyncMode::WALL_CLOCK
                                                : bitsim::distributed::SyncMode::LOCKSTEP;
        else if (flag == "--transport")
            options.transport = value;
        else if (flag == "--participant")
            options.participant = std::atoi(value.c_str());
        else if (flag == "--broker")
            options.broker = value;
        else
        {
            std::cerr << "unknown option " << flag << "\n";
            return 1;
        }
    }
    return distributed_chrono_robots::run(options);
}

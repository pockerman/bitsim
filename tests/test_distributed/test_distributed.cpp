#include "bitsim/bitsim_config.h"
#include "bitsim/distributed/state_message.h"
#include "bitsim/distributed/synchroniser.h"
#include "bitsim/distributed/transport.h"

#ifdef BITSIM_MQTT
#include "bitsim/distributed/mqtt_transport.h"
#endif

#ifdef BITSIM_CHRONO
#include "bitsim/distributed/chrono_ghosts.h"
#include "bitsim/sensors/backends/chrono_ultrasound_backend.h"
#include <chrono/physics/ChContactMaterialNSC.h>
#include <chrono/physics/ChSystemNSC.h>
#endif

#include <gtest/gtest.h>

#include <chrono>
#include <map>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{
using bitsim::real_t;
using bitsim::uint_t;
using namespace bitsim::distributed;
using namespace std::chrono_literals;

TEST(TestStateMessage, SurvivesEncoding)
{
    StateMessage m;
    m.participant = "robot_1";
    m.step = 17;
    m.time = 0.85;
    m.stamp = 1.5e9;
    m.bodies.push_back(
        BodyState::planar("chassis", 1.0, -2.0, 0.3, 0.5, 0.1, -0.2, {0.5, 0.3, 0.1}));
    const auto back = StateMessage::decode(m.encode());
    EXPECT_EQ(back.participant, "robot_1");
    EXPECT_EQ(back.step, 17u);
    EXPECT_DOUBLE_EQ(back.time, 0.85);
    ASSERT_EQ(back.bodies.size(), 1u);
    EXPECT_EQ(back.bodies[0].name, "chassis");
    EXPECT_DOUBLE_EQ(back.bodies[0].position[1], -2.0);
    EXPECT_NEAR(back.bodies[0].yaw(), 0.3, 1e-12);
    EXPECT_DOUBLE_EQ(back.bodies[0].size[0], 0.5);
    EXPECT_TRUE(back.values.empty());
    m.values = {3.0, -0.25};
    EXPECT_EQ(StateMessage::decode(m.encode()).values, m.values);
    EXPECT_THROW(StateMessage::decode("not msgpack at all"), std::invalid_argument);
}

TEST(TestBodyState, ExtrapolatesPositionAndHeading)
{
    const auto b = BodyState::planar("b", 0.0, 0.0, 0.0, 1.0, 0.5, 0.2);
    const auto later = b.extrapolated(2.0);
    EXPECT_DOUBLE_EQ(later.position[0], 2.0);
    EXPECT_DOUBLE_EQ(later.position[1], 1.0);
    EXPECT_NEAR(later.yaw(), 0.4, 1e-12);
    EXPECT_NEAR(b.extrapolated(0.0).yaw(), 0.0, 1e-15);
}

TEST(TestInProcessBus, DeliversToEveryoneElse)
{
    InProcessBus bus;
    auto a = bus.connect();
    auto b = bus.connect();
    auto c = bus.connect();
    a->publish("hello");
    EXPECT_EQ(b->receive(10ms), "hello");
    EXPECT_EQ(c->receive(10ms), "hello");
    EXPECT_EQ(a->receive(0ms), std::nullopt);
}

TEST(TestInProcessBus, ALateParticipantMissesWhatCameBefore)
{
    InProcessBus bus;
    auto a = bus.connect();
    a->publish("early");
    auto b = bus.connect();
    a->publish("late");
    EXPECT_EQ(b->receive(10ms), "late");
    EXPECT_EQ(b->receive(0ms), std::nullopt);
}

TEST(TestLossyTransport, DelaysAndLoses)
{
    InProcessBus bus;
    auto sender = bus.connect();
    LossyTransport::Config config;
    config.latency = 0.03;
    LossyTransport delayed(bus.connect(), config);
    const auto start = std::chrono::steady_clock::now();
    sender->publish("x");
    EXPECT_EQ(delayed.receive(0ms), std::nullopt); // not yet
    EXPECT_EQ(delayed.receive(200ms), "x");
    EXPECT_GE(std::chrono::steady_clock::now() - start, 25ms);

    config.latency = 0.0;
    config.loss = 1.0;
    LossyTransport lossy(bus.connect(), config);
    sender->publish("y");
    EXPECT_EQ(lossy.receive(20ms), std::nullopt);
    EXPECT_EQ(lossy.n_lost(), 1u);
}

// Three participants, each a thread, each moving a point; every step a participant moves
// towards the mean of the others, as the synchroniser reports them
std::map<std::string, std::vector<real_t>> run_consensus(SyncMode mode,
                                                         const LossyTransport::Config *network,
                                                         uint_t n_steps, uint_t late_start_ms = 0)
{
    InProcessBus bus;
    const std::vector<std::string> names{"a", "b", "c"};
    std::map<std::string, std::vector<real_t>> paths;
    std::vector<std::unique_ptr<Transport>> transports;
    for (std::size_t i = 0; i < names.size(); ++i)
    {
        auto t = bus.connect();
        if (network)
        {
            auto config = *network;
            config.seed += i;
            t = std::make_unique<LossyTransport>(std::move(t), config);
        }
        transports.push_back(std::move(t));
        paths[names[i]] = {};
    }

    std::vector<std::thread> threads;
    for (std::size_t i = 0; i < names.size(); ++i)
    {
        threads.emplace_back(
            [&, i]
            {
                if (i == 2 && late_start_ms)
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(late_start_ms));
                }
                SynchroniserConfig config;
                config.mode = mode;
                config.sync_period = 0.01;
                config.resend_period = 0.02;
                config.timeout = 5.0;
                for (const auto &n : names)
                {
                    if (n != names[i])
                    {
                        config.peers.push_back(n);
                    }
                }
                Synchroniser sync(*transports[i], names[i], config);
                real_t x = static_cast<real_t>(i);
                auto &path = paths[names[i]];
                for (uint_t k = 0; k < n_steps; ++k)
                {
                    const auto &peers = sync.exchange(k, {BodyState::planar("p", x, 0.0, 0.0)});
                    real_t mean = 0.0;
                    uint_t n = 0;
                    for (const auto &peer : peers)
                    {
                        if (!peer.bodies.empty())
                        {
                            mean += peer.bodies[0].position[0];
                            ++n;
                        }
                    }
                    if (n)
                    {
                        x += 0.5 * (mean / n - x);
                    }
                    path.push_back(x);
                }
                sync.leave();
            });
    }
    for (auto &t : threads)
    {
        t.join();
    }
    return paths;
}

TEST(TestSynchroniser, LockstepIsRepeatable)
{
    const auto first = run_consensus(SyncMode::LOCKSTEP, nullptr, 50);
    const auto second = run_consensus(SyncMode::LOCKSTEP, nullptr, 50);
    EXPECT_EQ(first, second);
    // they meet in the middle
    EXPECT_NEAR(first.at("a").back(), 1.0, 1e-9);
    EXPECT_NEAR(first.at("c").back(), 1.0, 1e-9);
}

TEST(TestSynchroniser, LockstepDoesNotDependOnTheNetwork)
{
    const auto clean = run_consensus(SyncMode::LOCKSTEP, nullptr, 30);
    LossyTransport::Config network;
    network.latency = 0.005;
    network.jitter = 0.005;
    network.loss = 0.2;
    const auto lossy = run_consensus(SyncMode::LOCKSTEP, &network, 30);
    EXPECT_EQ(clean, lossy);
}

TEST(TestSynchroniser, LockstepWaitsForALateParticipant)
{
    // c connects 200 ms after the others have sent their first step
    const auto late = run_consensus(SyncMode::LOCKSTEP, nullptr, 20, 200);
    const auto clean = run_consensus(SyncMode::LOCKSTEP, nullptr, 20);
    EXPECT_EQ(late, clean);
}

TEST(TestSynchroniser, LockstepCarriesValues)
{
    InProcessBus bus;
    auto ta = bus.connect();
    auto tb = bus.connect();
    SynchroniserConfig config;
    config.timeout = 2.0;
    config.peers = {"b"};
    Synchroniser a(*ta, "a", config);
    config.peers = {"a"};
    std::vector<real_t> b_got;
    std::thread b_thread(
        [&]
        {
            Synchroniser b(*tb, "b", config);
            b_got = b.exchange(0, {}, {1.0, 2.0}).front().values;
        });
    const auto &peers = a.exchange(0, {}, {5.0});
    b_thread.join();
    EXPECT_EQ(peers.front().values, std::vector<real_t>({1.0, 2.0}));
    EXPECT_EQ(b_got, std::vector<real_t>({5.0}));
}

TEST(TestSynchroniser, LockstepTimesOutNamingTheMissingPeer)
{
    InProcessBus bus;
    auto t = bus.connect();
    SynchroniserConfig config;
    config.peers = {"nobody"};
    config.timeout = 0.1;
    Synchroniser sync(*t, "me", config);
    try
    {
        sync.exchange(0, {});
        FAIL() << "expected a timeout";
    }
    catch (const std::runtime_error &e)
    {
        EXPECT_NE(std::string(e.what()).find("nobody"), std::string::npos);
    }
}

TEST(TestSynchroniser, LockstepStopsWaitingForAPeerThatLeft)
{
    InProcessBus bus;
    auto ta = bus.connect();
    auto tb = bus.connect();
    SynchroniserConfig config;
    config.timeout = 2.0;
    config.peers = {"b"};
    Synchroniser a(*ta, "a", config);
    config.peers = {"a"};
    std::thread b_thread(
        [&]
        {
            Synchroniser b(*tb, "b", config);
            b.exchange(0, {BodyState::planar("p", 7.0, 0.0, 0.0)});
            b.leave();
        });
    const auto &first = a.exchange(0, {});
    ASSERT_EQ(first.size(), 1u);
    EXPECT_DOUBLE_EQ(first[0].bodies.at(0).position[0], 7.0);
    b_thread.join();
    const auto &after = a.exchange(1, {}); // does not wait for b
    ASSERT_EQ(after.size(), 1u);
    EXPECT_TRUE(after[0].left);
    EXPECT_DOUBLE_EQ(after[0].bodies.at(0).position[0], 7.0); // its last state
}

TEST(TestSynchroniser, LockstepPeerThatLeftAfterAStepHasNotLeftAtIt)
{
    // b sends step 0 and leaves before a asks for step 0, so a receives both messages at once.
    // b did step 0: a gets its state for step 0, and b has left only for the steps after it
    InProcessBus bus;
    auto ta = bus.connect();
    auto tb = bus.connect();
    SynchroniserConfig config;
    config.timeout = 2.0;
    {
        Synchroniser b(*tb, "b", config); // no peers: it does not wait for a
        b.exchange(0, {BodyState::planar("p", 7.0, 0.0, 0.0)});
        b.leave();
    }
    config.peers = {"b"};
    Synchroniser a(*ta, "a", config);
    const auto &at_its_last_step = a.exchange(0, {});
    ASSERT_EQ(at_its_last_step.size(), 1u);
    EXPECT_FALSE(at_its_last_step[0].left);
    EXPECT_EQ(at_its_last_step[0].step, 0u);
    EXPECT_DOUBLE_EQ(at_its_last_step[0].bodies.at(0).position[0], 7.0);
    const auto &after = a.exchange(1, {});
    ASSERT_EQ(after.size(), 1u);
    EXPECT_TRUE(after[0].left);
}

TEST(TestSynchroniser, RejectsBadUse)
{
    InProcessBus bus;
    auto t = bus.connect();
    SynchroniserConfig config;
    config.peers = {"me"};
    EXPECT_THROW(Synchroniser(*t, "me", config), std::invalid_argument);
    config.peers = {};
    Synchroniser sync(*t, "me", config);
    sync.exchange(3, {});
    EXPECT_THROW(sync.exchange(3, {}), std::invalid_argument);
    sync.leave();
    EXPECT_THROW(sync.exchange(4, {}), std::logic_error);
}

TEST(TestSynchroniser, WallClockKeepsPace)
{
    InProcessBus bus;
    auto t = bus.connect();
    SynchroniserConfig config;
    config.mode = SyncMode::WALL_CLOCK;
    config.sync_period = 0.02;
    Synchroniser sync(*t, "me", config);
    const auto start = std::chrono::steady_clock::now();
    for (uint_t k = 0; k < 10; ++k)
    {
        sync.exchange(k, {});
    }
    const auto elapsed = std::chrono::steady_clock::now() - start;
    EXPECT_GE(elapsed, 175ms); // steps 0 to 9: 9 periods
    EXPECT_LT(elapsed, 400ms);
    EXPECT_EQ(sync.n_overruns(), 0u);
}

TEST(TestSynchroniser, WallClockExtrapolatesAndMarksStaleStates)
{
    InProcessBus bus;
    auto robot = bus.connect(); // stands in for a robot on the network
    auto sim = bus.connect();
    SynchroniserConfig config;
    config.mode = SyncMode::WALL_CLOCK;
    config.sync_period = 0.01;
    config.stale_after = 0.15;
    config.peers = {"robot"};
    Synchroniser sync(*sim, "sim", config);

    // nothing heard yet: stale, no bodies
    auto peers = sync.exchange(0, {});
    ASSERT_EQ(peers.size(), 1u);
    EXPECT_TRUE(peers[0].stale);
    EXPECT_TRUE(peers[0].bodies.empty());

    // the robot reports once, moving at 1 m/s along x
    StateMessage m;
    m.participant = "robot";
    m.stamp = wall_time_now();
    m.bodies.push_back(BodyState::planar("chassis", 0.0, 0.0, 0.0, 1.0));
    robot->publish(m.encode());
    std::this_thread::sleep_for(50ms);
    peers = sync.exchange(1, {});
    ASSERT_EQ(peers.size(), 1u);
    EXPECT_FALSE(peers[0].stale);
    EXPECT_GT(peers[0].age, 0.04);
    // moved forward by its age
    EXPECT_NEAR(peers[0].bodies[0].position[0], peers[0].age, 0.01);

    std::this_thread::sleep_for(200ms);
    peers = sync.exchange(30, {});
    EXPECT_TRUE(peers[0].stale);
    // a stale state moves no further than stale_after
    EXPECT_NEAR(peers[0].bodies[0].position[0], 0.15, 1e-9);
}

#ifdef BITSIM_MQTT
TEST(TestMQTT_Transport, TwoParticipantsInLockstep)
{
    std::unique_ptr<MQTT_Transport> ta, tb;
    try
    {
        ta = std::make_unique<MQTT_Transport>("tcp://localhost:1883", "test_distributed");
        tb = std::make_unique<MQTT_Transport>("tcp://localhost:1883", "test_distributed");
    }
    catch (const std::exception &e)
    {
        GTEST_SKIP() << "no MQTT broker on localhost:1883: " << e.what();
    }
    SynchroniserConfig config;
    config.timeout = 5.0;
    real_t got_by_a = 0.0;
    std::thread b_thread(
        [&]
        {
            SynchroniserConfig c = config;
            c.peers = {"a"};
            Synchroniser b(*tb, "b", c);
            for (uint_t k = 0; k < 5; ++k)
            {
                b.exchange(k, {BodyState::planar("p", 10.0 + k, 0.0, 0.0)});
            }
        });
    config.peers = {"b"};
    Synchroniser a(*ta, "a", config);
    for (uint_t k = 0; k < 5; ++k)
    {
        const auto &peers = a.exchange(k, {BodyState::planar("p", 0.0, 0.0, 0.0)});
        got_by_a = peers.at(0).bodies.at(0).position[0];
        EXPECT_DOUBLE_EQ(got_by_a, 10.0 + k);
    }
    b_thread.join();
}
#endif

#ifdef BITSIM_CHRONO
TEST(TestCHRONO_Ghosts, ASensorSeesTheGhostOfAnotherRobot)
{
    chrono::ChSystemNSC system;
    system.SetCollisionSystemType(chrono::ChCollisionSystem::Type::BULLET);
    auto material = chrono_types::make_shared<chrono::ChContactMaterialNSC>();
    auto own = chrono_types::make_shared<chrono::ChBodyEasyBox>(0.5, 0.3, 0.1, 100.0, false, true,
                                                                material);
    own->SetFixed(true);
    system.AddBody(own);
    bitsim::sensors::backends::CHRONO_UltrasonicBackend sonar(system, own);
    sonar.set_position(chrono::ChVector3d(0.25, 0.0, 0.0));
    nlohmann::json j = {{"min_distance", 0.02}, {"max_distance", 4.0}};
    sonar.load(j);

    CHRONO_Ghosts ghosts(system);
    PeerState peer;
    peer.participant = "robot_1";
    peer.bodies.push_back(
        BodyState::planar("chassis", 2.25, 0.0, 0.0, 0.0, 0.0, 0.0, {0.5, 0.3, 0.1}));
    ghosts.update({peer});
    system.DoStepDynamics(0.001);
    EXPECT_EQ(ghosts.size(), 1u);
    // the sonar at x = 0.25 sees the near face of the ghost at x = 2.0
    EXPECT_NEAR(sonar.read_values().at(0), 1.75, 0.01);

    // the other robot comes closer
    peer.bodies[0] = BodyState::planar("chassis", 1.25, 0.0, 0.0, 0.0, 0.0, 0.0, {0.5, 0.3, 0.1});
    ghosts.update({peer});
    system.DoStepDynamics(0.001);
    EXPECT_NEAR(sonar.read_values().at(0), 0.75, 0.01);

    // and leaves
    peer.left = true;
    ghosts.update({peer});
    system.DoStepDynamics(0.001);
    EXPECT_EQ(ghosts.size(), 0u);
    EXPECT_NEAR(sonar.read_values().at(0), 4.0, 1e-9);
}
#endif

} // namespace

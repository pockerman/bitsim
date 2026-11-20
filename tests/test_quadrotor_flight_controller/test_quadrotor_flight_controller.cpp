#include "bitsim/bitsim_config.h"
#include "bitsim/bitsim_consts.h"
#include "bitsim/bitsim_types.h"
#include "bitsim/boards/sim_rc_link.h"
#include "bitsim/control/quadrotor_flight_controller.h"
#include "bitsim/control/rc_command.h"
#include "bitsim/dynamics/quadrotor_dynamics.h"
#include "bitsim/dynamics/system_state.h"

#if defined(BITSIM_IRRLICHT) && defined(BITSIM_CHRONO)
#include "bitsim/utils/render/keyboard_rc.h"
#endif

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <string>
#include <utility>

namespace
{
using bitsim::real_t;
using bitsim::RealColVec3d;
using bitsim::RealVec;
using bitsim::boards::SIM_RCLink;
using bitsim::control::QuadrotorFlightController;
using bitsim::control::RCCommand;
using bitsim::dynamics::QuadrotorDynamics;
using bitsim::dynamics::QuadrotorDynamicsConfig;
using bitsim::dynamics::SysState;

const real_t DT = 0.001;

QuadrotorDynamicsConfig make_config()
{
    QuadrotorDynamicsConfig config;
    config.dt = DT;
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

SysState<12> level_state()
{
    std::array<std::pair<std::string, real_t>, 12> values{{{"x", 0.0},
                                                           {"y", 0.0},
                                                           {"z", 0.0},
                                                           {"u", 0.0},
                                                           {"v", 0.0},
                                                           {"w", 0.0},
                                                           {"p", 0.0},
                                                           {"q", 0.0},
                                                           {"r", 0.0},
                                                           {"phi", 0.0},
                                                           {"theta", 0.0},
                                                           {"psi", 0.0}}};
    return SysState<12>(std::move(values));
}

// Fly with a fixed command for time
void fly(QuadrotorDynamics &quadrotor, const QuadrotorFlightController &controller,
         const RCCommand &command, real_t time)
{
    const int n = static_cast<int>(std::round(time / DT));
    for (int k = 0; k < n; ++k)
    {
        quadrotor.integrate(controller.rotor_speeds(command, quadrotor));
    }
}

RCCommand hover(const QuadrotorFlightController &controller)
{
    RCCommand command;
    command.armed = true;
    command.throttle = controller.hover_throttle();
    return command;
}
} // namespace

TEST(TestQuadrotorMixer, GivesTheThrustAndTheTorques)
{
    const auto config = make_config();
    const RealColVec3d tau(0.01, -0.02, 0.003);
    const RealVec w = bitsim::control::quadrotor_mixer(config, 5.0, tau, 2000.0);
    RealVec s = w.cwiseProduct(w);

    EXPECT_NEAR(config.k_1 * s.sum(), 5.0, 1.0e-9);
    EXPECT_NEAR(config.l * config.k_1 * (s[3] - s[1]), tau[0], 1.0e-9);
    EXPECT_NEAR(config.l * config.k_1 * (s[0] - s[2]), tau[1], 1.0e-9);
    EXPECT_NEAR(config.k_2 * (s[0] - s[1] + s[2] - s[3]), tau[2], 1.0e-9);
}

TEST(TestQuadrotorMixer, ClampsTheRotorSpeeds)
{
    const auto config = make_config();
    const RealVec w = bitsim::control::quadrotor_mixer(config, 0.0, RealColVec3d(0.05, 0, 0), 700);
    EXPECT_DOUBLE_EQ(w[1], 0.0); // it would have to push down
    EXPECT_LE(w[3], 700.0);
}

TEST(TestSIMRCLink, FailsafeBeforeTheFirstCommandAndAfterTheTimeout)
{
    SIM_RCLink link(0.5);
    EXPECT_TRUE(link.in_failsafe(0.0));
    EXPECT_FALSE(link.receive(0.0).armed);

    RCCommand command;
    command.armed = true;
    command.throttle = 1.5; // out of range
    command.roll = -0.3;
    link.send(command, 1.0);
    EXPECT_FALSE(link.in_failsafe(1.4));
    EXPECT_TRUE(link.receive(1.4).armed);
    EXPECT_DOUBLE_EQ(link.receive(1.4).throttle, 1.0);
    EXPECT_DOUBLE_EQ(link.receive(1.4).roll, -0.3);

    EXPECT_TRUE(link.in_failsafe(1.6));
    EXPECT_FALSE(link.receive(1.6).armed);
    EXPECT_DOUBLE_EQ(link.receive(1.6).throttle, 0.0);
}

TEST(TestQuadrotorFlightController, DisarmedTheRotorsStop)
{
    const auto config = make_config();
    auto state = level_state();
    QuadrotorDynamics quadrotor(config, state);
    const QuadrotorFlightController controller(config);
    RCCommand command = hover(controller);
    command.armed = false;
    EXPECT_DOUBLE_EQ(controller.rotor_speeds(command, quadrotor).norm(), 0.0);
}

TEST(TestQuadrotorFlightController, HoversAtTheHoverThrottle)
{
    const auto config = make_config();
    auto state = level_state();
    QuadrotorDynamics quadrotor(config, state);
    const QuadrotorFlightController controller(config);
    EXPECT_DOUBLE_EQ(controller.hover_throttle(), 0.5);

    fly(quadrotor, controller, hover(controller), 3.0);
    EXPECT_NEAR(quadrotor.get_position().norm(), 0.0, 1.0e-9);
}

TEST(TestQuadrotorFlightController, RollStickTiltsItRightAndItFliesRight)
{
    const auto config = make_config();
    auto state = level_state();
    QuadrotorDynamics quadrotor(config, state);
    const QuadrotorFlightController controller(config);
    RCCommand command = hover(controller);
    command.roll = 0.5;
    fly(quadrotor, controller, command, 1.0);

    EXPECT_NEAR(quadrotor.phi(), 0.5 * controller.parameters().max_tilt, 0.01);
    EXPECT_NEAR(quadrotor.theta(), 0.0, 1.0e-3);
    EXPECT_GT(quadrotor.get_position()[1], 0.1); // east, to the right

    // Letting go levels it
    fly(quadrotor, controller, hover(controller), 1.0);
    EXPECT_NEAR(quadrotor.phi(), 0.0, 0.01);
}

TEST(TestQuadrotorFlightController, PitchStickForwardTiltsTheNoseDownAndItFliesForward)
{
    const auto config = make_config();
    auto state = level_state();
    QuadrotorDynamics quadrotor(config, state);
    const QuadrotorFlightController controller(config);
    RCCommand command = hover(controller);
    command.pitch = 0.5;
    fly(quadrotor, controller, command, 1.0);

    EXPECT_NEAR(quadrotor.theta(), -0.5 * controller.parameters().max_tilt, 0.01);
    EXPECT_GT(quadrotor.get_position()[0], 0.1); // north, ahead
}

TEST(TestQuadrotorFlightController, YawStickTurnsItClockwiseAtTheRate)
{
    const auto config = make_config();
    auto state = level_state();
    QuadrotorDynamics quadrotor(config, state);
    const QuadrotorFlightController controller(config);
    RCCommand command = hover(controller);
    command.yaw = 0.5;
    fly(quadrotor, controller, command, 2.0);

    EXPECT_NEAR(quadrotor.get_angular_velocity()[2], 0.5 * controller.parameters().max_yaw_rate,
                0.01);
    EXPECT_GT(quadrotor.psi(), 0.0);
}

#if defined(BITSIM_IRRLICHT) && defined(BITSIM_CHRONO)
TEST(TestKeyboardRC, KeysMoveTheSticks)
{
    bitsim::utils::render::KeyboardRC keyboard(0.25, 1.0);
    EXPECT_FALSE(keyboard.command(0.01).armed);

    keyboard.set_key(irr::KEY_SPACE, true);
    keyboard.set_key(irr::KEY_SPACE, true); // a repeat does not toggle again
    keyboard.set_key(irr::KEY_SPACE, false);
    keyboard.set_key(irr::KEY_KEY_W, true);
    RCCommand command;
    for (int k = 0; k < 200; ++k)
    {
        command = keyboard.command(0.01);
    }
    EXPECT_TRUE(command.armed);
    EXPECT_NEAR(command.throttle, 0.5, 1.0e-9);

    // The throttle stays where it is left; the other sticks spring back
    keyboard.set_key(irr::KEY_KEY_W, false);
    keyboard.set_key(irr::KEY_UP, true);
    keyboard.set_key(irr::KEY_LEFT, true);
    command = keyboard.command(0.01);
    EXPECT_NEAR(command.throttle, 0.5, 1.0e-9);
    EXPECT_DOUBLE_EQ(command.pitch, 1.0);
    EXPECT_DOUBLE_EQ(command.roll, -1.0);
    keyboard.set_key(irr::KEY_UP, false);
    keyboard.set_key(irr::KEY_LEFT, false);
    command = keyboard.command(0.01);
    EXPECT_DOUBLE_EQ(command.pitch, 0.0);
    EXPECT_DOUBLE_EQ(command.roll, 0.0);

    keyboard.set_key(irr::KEY_KEY_R, true);
    EXPECT_TRUE(keyboard.reset_requested());
    EXPECT_FALSE(keyboard.reset_requested());
}
#endif

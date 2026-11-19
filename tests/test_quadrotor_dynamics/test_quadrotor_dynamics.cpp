#include "bitrl/bitrl_types.h"
#include "bitrl/dynamics/quadrotor_dynamics.h"
#include "bitrl/dynamics/system_state.h"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <string>
#include <utility>

namespace
{

using bitrl::real_t;
using bitrl::RealColVec3d;
using bitrl::RealVec;
using bitrl::dynamics::QuadrotorDynamics;
using bitrl::dynamics::QuadrotorDynamicsConfig;
using bitrl::dynamics::SysState;

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
    config.use_gravity = false;
    return config;
}

SysState<12> make_state(real_t v, real_t phi, real_t theta, real_t psi)
{
    std::array<std::pair<std::string, real_t>, 12> values;
    values[0] = {"x", 0.0};
    values[1] = {"y", 0.0};
    values[2] = {"z", 0.0};
    values[3] = {"u", 0.0};
    values[4] = {"v", v};
    values[5] = {"w", 0.0};
    values[6] = {"p", 0.0};
    values[7] = {"q", 0.0};
    values[8] = {"r", 0.0};
    values[9] = {"phi", phi};
    values[10] = {"theta", theta};
    values[11] = {"psi", psi};
    return SysState<12>(std::move(values));
}

} // namespace

// The position advances by dt * R * v. R is orthogonal so the step has length dt * |v|
// whatever the attitude is. No motors, gravity or rotation, so the attitude stays fixed
TEST(TestQuadrotorDynamics, PositionStepPreservesTheLengthOfTheVelocity)
{
    const real_t speed = 2.0;
    auto state = make_state(speed, 0.3, 0.2, -0.4);
    QuadrotorDynamics dynamics(make_config(), state);

    dynamics.integrate(RealVec::Zero(4));

    EXPECT_NEAR(dynamics.get_position().norm(), DT * speed, 1.0e-12);
}

namespace
{
const real_t G = 9.81;

QuadrotorDynamicsConfig with_gravity()
{
    auto config = make_config();
    config.use_gravity = true;
    return config;
}

// The rotor speed whose thrust carries the weight: 4 k w^2 = m g
real_t hover_speed(const QuadrotorDynamicsConfig &config)
{
    return std::sqrt(config.mass * G / (4.0 * config.k_1));
}

RealVec rotors(real_t w1, real_t w2, real_t w3, real_t w4)
{
    RealVec w(4);
    w << w1, w2, w3, w4;
    return w;
}

void fly(QuadrotorDynamics &dynamics, const RealVec &w, real_t time)
{
    const int n = static_cast<int>(std::round(time / DT));
    for (int k = 0; k < n; ++k)
    {
        dynamics.integrate(w);
    }
}
} // namespace

TEST(TestQuadrotorDynamics, HoversAtTheHoverSpeed)
{
    const auto config = with_gravity();
    auto state = make_state(0.0, 0.0, 0.0, 0.0);
    QuadrotorDynamics dynamics(config, state);
    const real_t w = hover_speed(config);
    fly(dynamics, rotors(w, w, w, w), 2.0);

    EXPECT_NEAR(dynamics.get_position().norm(), 0.0, 1.0e-9);
    EXPECT_NEAR(dynamics.get_velocity().norm(), 0.0, 1.0e-9);
    EXPECT_NEAR(dynamics.get_euler_angles().norm(), 0.0, 1.0e-12);
}

TEST(TestQuadrotorDynamics, FallsWithTheRotorsStopped)
{
    auto state = make_state(0.0, 0.0, 0.0, 0.0);
    QuadrotorDynamics dynamics(with_gravity(), state);
    fly(dynamics, rotors(0.0, 0.0, 0.0, 0.0), 1.0);

    // z is down: after a second it has fallen g / 2, at a speed of g
    EXPECT_NEAR(dynamics.get_position()[2], 0.5 * G, 0.01);
    EXPECT_NEAR(dynamics.get_velocity()[2], G, 1.0e-9);
    EXPECT_NEAR(dynamics.get_position()[0], 0.0, 1.0e-12);
}

TEST(TestQuadrotorDynamics, ClimbsWithMoreThrust)
{
    const auto config = with_gravity();
    auto state = make_state(0.0, 0.0, 0.0, 0.0);
    QuadrotorDynamics dynamics(config, state);
    const real_t w = 1.1 * hover_speed(config);
    fly(dynamics, rotors(w, w, w, w), 1.0);

    // 1.21 times the weight leaves 0.21 g upwards, along -z
    const real_t a = 0.21 * G;
    EXPECT_NEAR(dynamics.get_velocity()[2], -a, 1.0e-6);
    EXPECT_NEAR(dynamics.get_position()[2], -0.5 * a, 0.005);
    EXPECT_NEAR(dynamics.get_euler_angles().norm(), 0.0, 1.0e-12);
}

TEST(TestQuadrotorDynamics, AFasterLeftRotorRollsItRightAndItDriftsRight)
{
    const auto config = with_gravity();
    auto state = make_state(0.0, 0.0, 0.0, 0.0);
    QuadrotorDynamics dynamics(config, state);
    const real_t w = hover_speed(config);
    const real_t dw = 5.0;
    fly(dynamics, rotors(w, w - dw, w, w + dw), 0.1);

    // tau = l k ((w + dw)^2 - (w - dw)^2) = 4 l k w dw, a constant angular acceleration
    const real_t p_dot = 4.0 * config.l * config.k_1 * w * dw / config.Jx;
    EXPECT_NEAR(dynamics.get_angular_velocity()[0], p_dot * 0.1, 1.0e-3 * p_dot * 0.1);
    EXPECT_GT(dynamics.phi(), 0.0);
    // The drag torques of rotors 2 and 4 no longer cancel, k2 (-2 dw^2): it yaws a little, and
    // the yaw couples a little of the roll into pitch
    EXPECT_NEAR(dynamics.theta(), 0.0, 1.0e-6);

    // Level the rotors again: it keeps rolling, the thrust is tilted to the right
    fly(dynamics, rotors(w, w, w, w), 0.2);
    EXPECT_GT(dynamics.get_position()[1], 1.0e-3);
    EXPECT_NEAR(dynamics.get_position()[0], 0.0, 1.0e-6);
}

TEST(TestQuadrotorDynamics, AFasterFrontRotorPitchesTheNoseUpAndItGoesBack)
{
    const auto config = with_gravity();
    auto state = make_state(0.0, 0.0, 0.0, 0.0);
    QuadrotorDynamics dynamics(config, state);
    const real_t w = hover_speed(config);
    fly(dynamics, rotors(w + 5.0, w, w - 5.0, w), 0.1);
    EXPECT_GT(dynamics.get_angular_velocity()[1], 0.0);
    EXPECT_GT(dynamics.theta(), 0.0);
    EXPECT_NEAR(dynamics.phi(), 0.0, 1.0e-6); // the same small yaw as for the roll

    fly(dynamics, rotors(w, w, w, w), 0.2);
    EXPECT_LT(dynamics.get_position()[0], -1.0e-3);
    EXPECT_NEAR(dynamics.get_position()[1], 0.0, 1.0e-6);
}

TEST(TestQuadrotorDynamics, FasterRotorsOneAndThreeYawItClockwise)
{
    const auto config = with_gravity();
    auto state = make_state(0.0, 0.0, 0.0, 0.0);
    QuadrotorDynamics dynamics(config, state);
    const real_t w = hover_speed(config);
    // The same total thrust, so it neither climbs nor falls
    const real_t up = std::sqrt(w * w + 1000.0);
    const real_t down = std::sqrt(w * w - 1000.0);
    fly(dynamics, rotors(up, down, up, down), 0.5);

    EXPECT_NEAR(dynamics.get_angular_velocity()[2], 4000.0 * config.k_2 / config.Jz * 0.5, 1.0e-9);
    EXPECT_GT(dynamics.psi(), 0.0);
    EXPECT_NEAR(dynamics.phi(), 0.0, 1.0e-12);
    EXPECT_NEAR(dynamics.theta(), 0.0, 1.0e-12);
    EXPECT_NEAR(dynamics.get_position().norm(), 0.0, 1.0e-9);
}

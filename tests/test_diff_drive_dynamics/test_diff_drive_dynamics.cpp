#include "bitsim/bitsim_consts.h"
#include "bitsim/bitsim_types.h"
#include "bitsim/dynamics/diff_drive_dynamics.h"
#include "bitsim/dynamics/system_state.h"

#include <gtest/gtest.h>

#include <any>
#include <array>
#include <cmath>
#include <map>
#include <string>

namespace
{
using bitsim::real_t;
using bitsim::dynamics::DiffDriveDynamics;
using bitsim::dynamics::SysState;

const real_t PI = bitsim::consts::maths::PI;
const real_t TOL = 1.0e-8;
const std::array<real_t, 2> NO_ERRORS{0.0, 0.0};

std::map<std::string, std::any> input(real_t v, real_t w, std::array<real_t, 2> errors = NO_ERRORS)
{
    return {{"v", std::any(v)}, {"w", std::any(w)}, {"errors", std::any(errors)}};
}

// Drive for time at v and w, in steps of dt, and return where the robot ends up
SysState<3> drive(real_t v, real_t w, real_t time, real_t dt)
{
    DiffDriveDynamics robot(DiffDriveDynamics::DynamicVersion::V1, false);
    robot.set_time_step(dt);
    const int n = static_cast<int>(std::round(time / dt));
    for (int k = 0; k < n; ++k)
    {
        robot.integrate(input(v, w));
    }
    return robot.get_state();
}

SysState<3> state(real_t x, real_t y, real_t theta)
{
    SysState<3> s;
    s.set(0, {"X", x});
    s.set(1, {"Y", y});
    s.set(2, {"Theta", theta});
    return s;
}
} // namespace

TEST(TestDiffDriveDynamics, StraightAhead)
{
    const auto robot = drive(0.5, 0.0, 2.0, 0.1);
    EXPECT_NEAR(robot[0], 1.0, 1.0e-12);
    EXPECT_NEAR(robot[1], 0.0, 1.0e-12);
    EXPECT_NEAR(robot[2], 0.0, 1.0e-12);
}

TEST(TestDiffDriveDynamics, AQuarterCircleIsExactWhateverTheStep)
{
    // 1 m/s turning at pi/2 rad/s: a circle of radius 2/pi, a quarter of it in a second
    const real_t R = 2.0 / PI;
    for (const real_t dt : {1.0, 0.1, 0.01})
    {
        const auto robot = drive(1.0, PI / 2.0, 1.0, dt);
        EXPECT_NEAR(robot[0], R, 1.0e-9) << "dt = " << dt;
        EXPECT_NEAR(robot[1], R, 1.0e-9) << "dt = " << dt;
        EXPECT_NEAR(robot[2], PI / 2.0, 1.0e-9) << "dt = " << dt;
    }
}

TEST(TestDiffDriveDynamics, AFullCircleComesBackWithTheHeadingWrapped)
{
    const auto robot = drive(1.0, -PI / 2.0, 4.0, 0.01);
    EXPECT_NEAR(robot[0], 0.0, 1.0e-9);
    EXPECT_NEAR(robot[1], 0.0, 1.0e-9);
    EXPECT_NEAR(std::sin(robot[2]), 0.0, 1.0e-9);
    EXPECT_LE(std::abs(robot[2]), PI + 1.0e-12);

    // Half way round, turning to the right, it is 2R to the right of where it started
    const auto half = drive(1.0, -PI / 2.0, 2.0, 0.01);
    EXPECT_NEAR(half[1], -4.0 / PI, 1.0e-9);
    EXPECT_NEAR(std::abs(half[2]), PI, 1.0e-9);
}

TEST(TestDiffDriveDynamics, JacobiansMatchFiniteDifferences)
{
    const real_t dt = 0.1;
    const real_t h = 1.0e-6;
    for (const real_t w : {0.0, 0.8})
    {
        const std::array<real_t, 2> errors{0.01, -0.02};
        const auto s0 = state(1.0, -0.5, 0.7);

        DiffDriveDynamics robot(DiffDriveDynamics::DynamicVersion::V1, true);
        robot.set_time_step(dt);
        robot.get_state() = s0;
        robot.initialize_matrices(input(0.6, w, errors));
        const auto F = robot.get_matrix("F");
        const auto L = robot.get_matrix("L");

        const auto step = [&](const SysState<3> &s, const std::array<real_t, 2> &e)
        { return DiffDriveDynamics::integrate_state_v1(s, TOL, dt, 0.6, w, e); };

        for (int j = 0; j < 3; ++j)
        {
            auto plus = s0;
            auto minus = s0;
            plus[j] += h;
            minus[j] -= h;
            const auto a = step(plus, errors);
            const auto b = step(minus, errors);
            for (int i = 0; i < 3; ++i)
            {
                EXPECT_NEAR(F(i, j), (a[i] - b[i]) / (2 * h), 1.0e-6)
                    << "F(" << i << ", " << j << ") with w = " << w;
            }
        }
        for (int j = 0; j < 2; ++j)
        {
            auto plus = errors;
            auto minus = errors;
            plus[j] += h;
            minus[j] -= h;
            const auto a = step(s0, plus);
            const auto b = step(s0, minus);
            for (int i = 0; i < 3; ++i)
            {
                EXPECT_NEAR(L(i, j), (a[i] - b[i]) / (2 * h), 1.0e-6)
                    << "L(" << i << ", " << j << ") with w = " << w;
            }
        }
    }
}

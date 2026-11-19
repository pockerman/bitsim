#include "bitrl/bitrl_consts.h"
#include "bitrl/bitrl_types.h"
#include "bitrl/dynamics/bicycle_vehicle_model_dynamics.h"
#include "bitrl/dynamics/system_state.h"

#include <gtest/gtest.h>

#include <any>
#include <array>
#include <cmath>
#include <map>
#include <string>

namespace
{
using bitrl::real_t;
using bitrl::dynamics::BicycleVehicleModelDynamics;
using bitrl::dynamics::BicycleVehicleModelDynamicsConfig;
using bitrl::dynamics::SysState;

const real_t PI = bitrl::consts::maths::PI;

std::map<std::string, std::any> input(real_t v, real_t delta, std::array<real_t, 2> errors = {})
{
    return {{"v", std::any(v)}, {"delta", std::any(delta)}, {"errors", std::any(errors)}};
}

SysState<3> state(real_t x, real_t y, real_t theta)
{
    SysState<3> s({"X", "Y", "Theta"}, 0.0);
    s[0] = x;
    s[1] = y;
    s[2] = theta;
    return s;
}
} // namespace

TEST(TestBicycleDynamics, DrivesStraightWithTheWheelStraight)
{
    auto s = state(1.0, 2.0, PI / 4.0);
    BicycleVehicleModelDynamics model({0.01, 0.5}, s);
    for (int k = 0; k < 100; ++k)
    {
        model.evaluate(input(2.0, 0.0));
    }
    // 2 m along the heading of 45 degrees, which does not change
    EXPECT_NEAR(model.get_state()[0], 1.0 + std::sqrt(2.0), 1.0e-9);
    EXPECT_NEAR(model.get_state()[1], 2.0 + std::sqrt(2.0), 1.0e-9);
    EXPECT_NEAR(model.get_state()[2], PI / 4.0, 1.0e-12);
}

TEST(TestBicycleDynamics, TurnsAtVTanDeltaOverL)
{
    // The heading changes at v tan(delta) / L, so a full circle takes 2 pi L / (v tan(delta))
    const real_t L = 0.5;
    const real_t v = 1.0;
    const real_t delta = 0.3;
    const real_t dt = 1.0e-4;
    auto s = state(0.0, 0.0, 0.0);
    BicycleVehicleModelDynamics model({dt, L}, s);
    const int n = static_cast<int>(std::round(2.0 * PI * L / (v * std::tan(delta)) / dt));
    for (int k = 0; k < n; ++k)
    {
        model.evaluate(input(v, delta));
    }
    EXPECT_NEAR(model.get_state()[2], 2.0 * PI, 1.0e-3);
    // Back where it started, after a circle of radius L / tan(delta)
    EXPECT_NEAR(model.get_state()[0], 0.0, 1.0e-3);
    EXPECT_NEAR(model.get_state()[1], 0.0, 1.0e-3);
}

TEST(TestBicycleDynamics, JacobiansMatchFiniteDifferences)
{
    const BicycleVehicleModelDynamicsConfig config{0.1, 0.5};
    const auto u = input(1.5, 0.2, {0.05, -0.02});
    const std::array<real_t, 3> x0{0.3, -0.4, 0.7};

    auto s = state(x0[0], x0[1], x0[2]);
    BicycleVehicleModelDynamics model(config, s);
    model.initialize_matrices(u);
    model.evaluate(u);
    const auto F = model.get_matrix("F");
    const auto L = model.get_matrix("L");

    // One step from a state, with the errors e
    const auto step = [&](std::array<real_t, 3> x, std::array<real_t, 2> e)
    {
        auto si = state(x[0], x[1], x[2]);
        BicycleVehicleModelDynamics m(config, si);
        m.set_matrix_update_flag(false);
        m.evaluate(input(1.5, 0.2, e));
        return std::array<real_t, 3>{m.get_state()[0], m.get_state()[1], m.get_state()[2]};
    };
    const real_t h = 1.0e-6;
    for (int j = 0; j < 3; ++j)
    {
        auto plus = x0;
        auto minus = x0;
        plus[j] += h;
        minus[j] -= h;
        const auto a = step(plus, {0.05, -0.02});
        const auto b = step(minus, {0.05, -0.02});
        for (int i = 0; i < 3; ++i)
        {
            EXPECT_NEAR(F(i, j), (a[i] - b[i]) / (2.0 * h), 1.0e-7)
                << "F(" << i << ", " << j << ")";
        }
    }
    for (int j = 0; j < 2; ++j)
    {
        std::array<real_t, 2> plus{0.05, -0.02};
        std::array<real_t, 2> minus{0.05, -0.02};
        plus[j] += h;
        minus[j] -= h;
        const auto a = step(x0, plus);
        const auto b = step(x0, minus);
        for (int i = 0; i < 3; ++i)
        {
            EXPECT_NEAR(L(i, j), (a[i] - b[i]) / (2.0 * h), 1.0e-7)
                << "L(" << i << ", " << j << ")";
        }
    }
}

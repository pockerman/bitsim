#include "bitrl/bitrl_types.h"
#include "bitrl/control/pid_controller.h"

#include <gtest/gtest.h>

#include <stdexcept>

namespace
{
using bitrl::real_t;
using bitrl::control::PIDController;
} // namespace

TEST(TestPIDController, ProportionalIntegralAndDerivativeTerms)
{
    PIDController p(2.0, 0.0, 0.0);
    EXPECT_DOUBLE_EQ(p.compute_ctrl_signal(1.0, 3.0, 0.1), 4.0);

    PIDController i(0.0, 1.0, 0.0);
    EXPECT_DOUBLE_EQ(i.compute_ctrl_signal(0.0, 1.0, 0.5), 0.5);
    EXPECT_DOUBLE_EQ(i.compute_ctrl_signal(0.0, 1.0, 0.5), 1.0);

    PIDController d(0.0, 0.0, 1.0);
    // No kick on the first call: there is no earlier error to take a derivative from
    EXPECT_DOUBLE_EQ(d.compute_ctrl_signal(0.0, 10.0, 0.1), 0.0);
    // The error goes from 10 to 8 in 0.1 s
    EXPECT_DOUBLE_EQ(d.compute_ctrl_signal(2.0, 10.0, 0.1), -20.0);
}

TEST(TestPIDController, LimitsTheOutputAndDoesNotWindUp)
{
    PIDController pid(1.0, 10.0, 0.0);
    pid.set_control_limits({-1.0, 1.0});
    EXPECT_TRUE(pid.is_limited());

    // A large error for a long time: the output stays at the limit, the integral does not grow
    for (int k = 0; k < 100; ++k)
    {
        EXPECT_DOUBLE_EQ(pid.compute_ctrl_signal(0.0, 5.0, 0.1), 1.0);
    }
    EXPECT_DOUBLE_EQ(pid.integral(), 0.0);

    // So once the error changes sign the output follows at once
    EXPECT_LT(pid.compute_ctrl_signal(5.5, 5.0, 0.1), 0.0);
}

TEST(TestPIDController, DrivesAFirstOrderSystemToTheTarget)
{
    // dx/dt = -x + u: a PI controller takes it to 2 with no error left
    PIDController pid(2.0, 1.0, 0.05);
    real_t x = 0.0;
    const real_t dt = 0.01;
    for (int k = 0; k < 2000; ++k)
    {
        const real_t u = pid.compute_ctrl_signal(x, 2.0, dt);
        x += (-x + u) * dt;
    }
    EXPECT_NEAR(x, 2.0, 1.0e-3);
}

TEST(TestPIDController, ResetForgetsTheIntegralAndTheLastError)
{
    PIDController pid(0.0, 1.0, 1.0);
    pid.compute_ctrl_signal(0.0, 1.0, 1.0);
    pid.compute_ctrl_signal(0.0, 3.0, 1.0);
    pid.reset();
    EXPECT_DOUBLE_EQ(pid.integral(), 0.0);
    // No derivative from before the reset
    EXPECT_DOUBLE_EQ(pid.compute_ctrl_signal(0.0, 1.0, 1.0), 1.0);
}

TEST(TestPIDController, RejectsBadArguments)
{
    PIDController pid(1.0, 0.0, 0.0);
    EXPECT_THROW(pid.set_control_limits({1.0, -1.0}), std::invalid_argument);
    EXPECT_THROW(pid.compute_ctrl_signal(0.0, 1.0, 0.0), std::invalid_argument);
}

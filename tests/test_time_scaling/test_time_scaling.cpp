#include "bitsim/bitsim_types.h"
#include "bitsim/trajectory/time_scaling.h"

#include <gtest/gtest.h>

#include <memory>
#include <stdexcept>

namespace
{
using bitsim::real_t;
using namespace bitsim::trajectory;

// s from 0 to 1, and its rates the derivatives of s and of each other
void check_consistent(const TimeScaling &timing)
{
    const real_t T = timing.duration();
    EXPECT_NEAR(timing.s(0.0), 0.0, 1.0e-12);
    EXPECT_NEAR(timing.s(T), 1.0, 1.0e-12);
    EXPECT_NEAR(timing.s(-1.0), 0.0, 1.0e-12);
    EXPECT_NEAR(timing.s(T + 1.0), 1.0, 1.0e-12);
    EXPECT_NEAR(timing.ds(0.0), 0.0, 1.0e-12);
    EXPECT_NEAR(timing.ds(T), 0.0, 1.0e-12);

    const real_t h = 1.0e-6;
    for (real_t t : {0.13 * T, 0.37 * T, 0.5 * T, 0.81 * T})
    {
        EXPECT_NEAR(timing.ds(t), (timing.s(t + h) - timing.s(t - h)) / (2 * h), 1.0e-6);
        EXPECT_NEAR(timing.dds(t), (timing.ds(t + h) - timing.ds(t - h)) / (2 * h), 1.0e-4);
    }
}
} // namespace

TEST(TestTimeScaling, Cubic)
{
    const CubicTimeScaling cubic(2.0);
    check_consistent(cubic);
    EXPECT_NEAR(cubic.ds(1.0), 1.5 / 2.0, 1.0e-12);  // peak at the middle
    EXPECT_NEAR(cubic.dds(0.0), 6.0 / 4.0, 1.0e-12); // the acceleration jumps at the start
    EXPECT_THROW(CubicTimeScaling(0.0), std::invalid_argument);
}

TEST(TestTimeScaling, Quintic)
{
    const QuinticTimeScaling quintic(2.0);
    check_consistent(quintic);
    EXPECT_NEAR(quintic.ds(1.0), 1.875 / 2.0, 1.0e-12);
    EXPECT_NEAR(quintic.dds(0.0), 0.0, 1.0e-12);
    EXPECT_NEAR(quintic.dds(2.0), 0.0, 1.0e-12);
}

TEST(TestTimeScaling, Trapezoidal)
{
    const auto trapezoid = TrapezoidalTimeScaling::with_accel_share(2.0, 0.25);
    check_consistent(trapezoid);
    EXPECT_NEAR(trapezoid.accel_time(), 0.5, 1.0e-12);
    EXPECT_NEAR(trapezoid.cruise_speed(), 1.0 / 1.5, 1.0e-12);
    EXPECT_NEAR(trapezoid.ds(1.0), trapezoid.cruise_speed(), 1.0e-12);
    EXPECT_NEAR(trapezoid.dds(1.0), 0.0, 1.0e-12);

    // With no cruise it is a triangle
    const TrapezoidalTimeScaling triangle(2.0, 1.0);
    EXPECT_NEAR(triangle.accel_time(), 1.0, 1.0e-12);
    EXPECT_NEAR(triangle.s(1.0), 0.5, 1.0e-12);

    EXPECT_THROW(TrapezoidalTimeScaling(2.0, 0.4), std::invalid_argument);
    EXPECT_THROW(TrapezoidalTimeScaling(2.0, 1.1), std::invalid_argument);
}

TEST(TestTimeScaling, PointToPoint)
{
    const PointToPoint motion({0.0, 1.0}, {2.0, -1.0}, std::make_shared<QuinticTimeScaling>(4.0));
    EXPECT_DOUBLE_EQ(motion.position(0.0)[0], 0.0);
    EXPECT_DOUBLE_EQ(motion.position(4.0)[1], -1.0);
    EXPECT_NEAR(motion.position(2.0)[0], 1.0, 1.0e-12);
    EXPECT_NEAR(motion.velocity(2.0)[1], -2.0 * 1.875 / 4.0, 1.0e-12);
    EXPECT_NEAR(motion.acceleration(0.0)[0], 0.0, 1.0e-12);
    EXPECT_THROW(PointToPoint({0.0}, {1.0, 2.0}, std::make_shared<CubicTimeScaling>(1.0)),
                 std::invalid_argument);
}

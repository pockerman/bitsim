#include "bitsim/bitsim_consts.h"
#include "bitsim/bitsim_types.h"
#include "bitsim/systems/double_pendulum_system.h"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <stdexcept>

namespace
{
using bitsim::real_t;
using bitsim::systems::DoublePendulumProperties;
using bitsim::systems::DoublePendulumSystem;

const real_t G = bitsim::consts::maths::G;

// The equations of motion of a double pendulum of two uniform rods, from its Lagrangian:
//   M11 a1 + M12 a2 = -m2 l1 c2 sin(t1 - t2) w2^2 - (m1 c1 + m2 l1) g sin t1
//   M12 a1 + M22 a2 =  m2 l1 c2 sin(t1 - t2) w1^2 - m2 c2 g sin t2
// with c the distance of a rod's centre from its pivot, I = m l^2 / 12 about its centre,
// M11 = I1 + m1 c1^2 + m2 l1^2, M12 = m2 l1 c2 cos(t1 - t2) and M22 = I2 + m2 c2^2
using State = std::array<real_t, 4>; // t1, t2, w1, w2

State derivative(const DoublePendulumProperties &p, const State &s)
{
    const real_t c1 = 0.5 * p.l1;
    const real_t c2 = 0.5 * p.l2;
    const real_t I1 = p.m1 * p.l1 * p.l1 / 12.0;
    const real_t I2 = p.m2 * p.l2 * p.l2 / 12.0;
    const real_t d = s[0] - s[1];
    const real_t M11 = I1 + p.m1 * c1 * c1 + p.m2 * p.l1 * p.l1;
    const real_t M12 = p.m2 * p.l1 * c2 * std::cos(d);
    const real_t M22 = I2 + p.m2 * c2 * c2;
    const real_t b1 = -p.m2 * p.l1 * c2 * std::sin(d) * s[3] * s[3] -
                      (p.m1 * c1 + p.m2 * p.l1) * G * std::sin(s[0]);
    const real_t b2 = p.m2 * p.l1 * c2 * std::sin(d) * s[2] * s[2] - p.m2 * c2 * G * std::sin(s[1]);
    const real_t det = M11 * M22 - M12 * M12;
    return {s[2], s[3], (M22 * b1 - M12 * b2) / det, (M11 * b2 - M12 * b1) / det};
}

State rk4(const DoublePendulumProperties &p, State s, real_t dt, int n)
{
    for (int k = 0; k < n; ++k)
    {
        const auto add = [](const State &a, const State &b, real_t h) {
            return State{a[0] + h * b[0], a[1] + h * b[1], a[2] + h * b[2], a[3] + h * b[3]};
        };
        const State k1 = derivative(p, s);
        const State k2 = derivative(p, add(s, k1, 0.5 * dt));
        const State k3 = derivative(p, add(s, k2, 0.5 * dt));
        const State k4 = derivative(p, add(s, k3, dt));
        for (int i = 0; i < 4; ++i)
        {
            s[i] += dt / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
        }
    }
    return s;
}
} // namespace

TEST(TestDoublePendulum, StartsWhereItIsPutAndHangsStill)
{
    DoublePendulumSystem pendulum({}, 0.4, -0.3);
    EXPECT_NEAR(pendulum.angles()[0], 0.4, 1.0e-12);
    EXPECT_NEAR(pendulum.angles()[1], -0.3, 1.0e-12);

    DoublePendulumSystem still;
    for (int k = 0; k < 1000; ++k)
    {
        still.step(1.0e-3);
    }
    EXPECT_NEAR(still.angles()[0], 0.0, 1.0e-9);
    EXPECT_NEAR(still.angles()[1], 0.0, 1.0e-9);
}

TEST(TestDoublePendulum, FollowsTheEquationsOfMotion)
{
    const DoublePendulumProperties props{0.8, 0.5, 1.0, 0.6, 0.04};
    DoublePendulumSystem pendulum(props, 0.6, 0.2);
    const real_t dt = 5.0e-4;
    const int n = 1000; // 0.5 s
    for (int k = 0; k < n; ++k)
    {
        pendulum.step(dt);
    }
    const State expected = rk4(props, {0.6, 0.2, 0.0, 0.0}, dt, n);
    EXPECT_NEAR(pendulum.angles()[0], expected[0], 5.0e-3);
    EXPECT_NEAR(pendulum.angles()[1], expected[1], 5.0e-3);
    EXPECT_NEAR(pendulum.angular_velocities()[0], expected[2], 2.0e-2);
    EXPECT_NEAR(pendulum.angular_velocities()[1], expected[3], 2.0e-2);
}

TEST(TestDoublePendulum, KeepsItsEnergy)
{
    // From horizontal: the links swing through a large motion, without friction
    const real_t horizontal = 0.5 * bitsim::consts::maths::PI;
    DoublePendulumSystem pendulum({}, horizontal, horizontal);
    const real_t e0 = pendulum.energy();
    // Both rods horizontal and still: their centres are at the height of the pivot
    EXPECT_NEAR(e0, 0.0, 1.0e-9);
    for (int k = 0; k < 1000; ++k) // 1 s
    {
        pendulum.step(1.0e-3);
    }
    EXPECT_GT(std::abs(pendulum.angles()[0] - horizontal), 0.5); // it has swung
    EXPECT_NEAR(pendulum.energy(), e0, 0.01 * 2.0 * G);          // 1 % of m g l
}

TEST(TestDoublePendulum, RejectsBadArguments)
{
    EXPECT_THROW(DoublePendulumSystem({0.0, 1.0, 1.0, 1.0, 0.04}), std::invalid_argument);
    DoublePendulumSystem pendulum;
    EXPECT_THROW(pendulum.step(0.0), std::invalid_argument);
}

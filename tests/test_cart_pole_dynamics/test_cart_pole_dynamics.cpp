// The cart-pole: what it conserves, its linearisation at upright, and an LQR that balances it
#include "bitsim/control/lqr.h"
#include "bitsim/dynamics/cart_pole_dynamics.h"

#include <gtest/gtest.h>

#include <cmath>

namespace
{
using bitsim::dynamics::CartPoleDynamics;
using bitsim::dynamics::CartPoleParameters;
using bitsim::dynamics::CartPoleState;
namespace cp = bitsim::dynamics::cart_pole;

// the horizontal momentum of the cart and the pole
double momentum(const CartPoleParameters &p, const CartPoleState &x)
{
    return (p.cart_mass + p.pole_mass) * x[cp::X_DOT] +
           p.pole_mass * p.pole_half_length * x[cp::THETA_DOT] * std::cos(x[cp::THETA]);
}
} // namespace

TEST(TestCartPoleDynamics, WithoutAForceItConservesEnergyAndMomentum)
{
    CartPoleState x0;
    x0 << 0.0, 0.3, 2.5, -1.0; // far from upright, swinging
    CartPoleDynamics cart_pole(CartPoleParameters(), x0);
    const double energy = cart_pole.energy(x0);
    const double p0 = momentum(cart_pole.parameters(), x0);

    for (int k = 0; k < 2000; ++k)
    {
        cart_pole.step(0.0, 0.001);
    }
    EXPECT_NEAR(cart_pole.energy(cart_pole.state()), energy, 1e-8);
    EXPECT_NEAR(momentum(cart_pole.parameters(), cart_pole.state()), p0, 1e-10);
    EXPECT_NEAR(cart_pole.time(), 2.0, 1e-9);
}

TEST(TestCartPoleDynamics, UprightIsAnEquilibrium)
{
    CartPoleDynamics cart_pole;
    EXPECT_LT(cart_pole.derivative(CartPoleState::Zero(), 0.0).norm(), 1e-15);

    // a small lean grows: it is unstable
    CartPoleState x = CartPoleState::Zero();
    x[cp::THETA] = 0.01;
    cart_pole.state() = x;
    for (int k = 0; k < 500; ++k)
    {
        cart_pole.step(0.0, 0.002);
    }
    EXPECT_GT(cart_pole.state()[cp::THETA], 0.1);
}

TEST(TestCartPoleDynamics, LinearisationAtUpright)
{
    const CartPoleParameters p;
    const CartPoleDynamics cart_pole(p);
    const auto model = cart_pole.linearise();

    const double total = p.cart_mass + p.pole_mass;
    const double l = p.pole_half_length;
    const double den = l * (4.0 / 3.0 - p.pole_mass / total);
    const double a_theta = p.gravity / den;      // d theta_acc / d theta
    const double b_theta = -1.0 / (total * den); // d theta_acc / d F

    EXPECT_NEAR(model.A(cp::X, cp::X_DOT), 1.0, 1e-6);
    EXPECT_NEAR(model.A(cp::THETA, cp::THETA_DOT), 1.0, 1e-6);
    EXPECT_NEAR(model.A(cp::THETA_DOT, cp::THETA), a_theta, 1e-5);
    EXPECT_NEAR(model.B[cp::THETA_DOT], b_theta, 1e-6);
    EXPECT_NEAR(model.A(cp::X_DOT, cp::THETA), -p.pole_mass * l * a_theta / total, 1e-5);
    EXPECT_NEAR(model.B[cp::X_DOT], 1.0 / total - p.pole_mass * l * b_theta / total, 1e-6);
}

TEST(TestCartPoleDynamics, TheForceIsLimited)
{
    CartPoleDynamics a;
    CartPoleDynamics b;
    a.step(1000.0, 0.01);
    b.step(a.parameters().max_force, 0.01);
    EXPECT_NEAR((a.state() - b.state()).norm(), 0.0, 1e-15);
}

TEST(TestCartPoleDynamics, AnLQRBalancesIt)
{
    CartPoleState x0 = CartPoleState::Zero();
    x0[cp::THETA] = 0.3;
    CartPoleDynamics cart_pole(CartPoleParameters(), x0);
    const auto model = cart_pole.linearise();

    const double dt = 0.02;
    const auto discrete = bitsim::control::discretise(model.A, model.B, dt);
    Eigen::Vector4d q;
    q << 1.0, 0.1, 10.0, 0.1;
    const auto lqr = bitsim::control::dlqr(discrete.A, discrete.B, q.asDiagonal().toDenseMatrix(),
                                           0.1 * Eigen::MatrixXd::Identity(1, 1));
    bitsim::control::LQRController controller(lqr.K);

    for (int k = 0; k < 500; ++k)
    {
        const double F = controller.control(cart_pole.state())[0];
        cart_pole.step(F, dt);
        ASSERT_LT(std::abs(cart_pole.state()[cp::THETA]), 0.5);
        ASSERT_FALSE(cart_pole.off_track());
    }
    EXPECT_LT(cart_pole.state().norm(), 1e-3);
}

// Model predictive control: without active bounds it is the LQR, and with them it keeps to them
#include "bitsim/control/linear_mpc.h"
#include "bitsim/control/lqr.h"

#include <gtest/gtest.h>

#include <limits>

namespace
{
using bitsim::control::LinearMPC;
using bitsim::control::Matrix;
using bitsim::control::Vector;

constexpr double INF = std::numeric_limits<double>::infinity();

// the double integrator, discretised for 0.1 s
bitsim::control::DiscreteLinearModel model()
{
    const Matrix A = (Matrix(2, 2) << 0.0, 1.0, 0.0, 0.0).finished();
    const Matrix B = (Matrix(2, 1) << 0.0, 1.0).finished();
    return bitsim::control::discretise(A, B, 0.1);
}

LinearMPC::QPSettings tight()
{
    LinearMPC::QPSettings settings;
    settings.eps_abs = 1e-9;
    settings.eps_rel = 1e-9;
    settings.max_iterations = 20000;
    return settings;
}
} // namespace

TEST(TestLinearMPC, WithoutActiveBoundsItIsTheLQR)
{
    const auto m = model();
    const Matrix Q = Matrix::Identity(2, 2);
    const Matrix R = Matrix::Identity(1, 1);

    // with the Riccati terminal weight, the plan's first input is the LQR's, for any horizon
    LinearMPC mpc(2, 1, 10, tight());
    mpc.set_model(m.A, m.B);
    mpc.set_weights(Q, R);
    const auto lqr = bitsim::control::dlqr(m.A, m.B, Q, R);
    EXPECT_LT((mpc.terminal_weight() - lqr.P).norm(), 1e-8);

    Vector x0(2);
    x0 << 1.0, -0.5;
    const Vector u = mpc.solve(x0);
    EXPECT_EQ(mpc.status(), LinearMPC::QPStatus::SOLVED);
    EXPECT_NEAR(u[0], (-lqr.K * x0)[0], 1e-5);
}

TEST(TestLinearMPC, KeepsToTheBounds)
{
    const auto m = model();
    LinearMPC mpc(2, 1, 30);
    mpc.set_model(m.A, m.B);
    mpc.set_weights(Matrix::Identity(2, 2), 0.01 * Matrix::Identity(1, 1));
    mpc.set_input_bounds(Vector::Constant(1, -0.5), Vector::Constant(1, 0.5));
    Vector x_min(2);
    x_min << -INF, -0.4;
    Vector x_max(2);
    x_max << INF, 0.4;
    mpc.set_state_bounds(x_min, x_max);

    // from 2 m away, at rest: it has to accelerate, coast at the speed limit and brake
    Vector x(2);
    x << 2.0, 0.0;
    for (int k = 0; k < 200; ++k)
    {
        const Vector u = mpc.solve(x);
        ASSERT_EQ(mpc.status(), LinearMPC::QPStatus::SOLVED) << "at step " << k;
        EXPECT_LE(std::abs(u[0]), 0.5 + 1e-4);
        for (const auto &xk : mpc.predicted_states())
        {
            EXPECT_LE(std::abs(xk[1]), 0.4 + 1e-3);
        }
        x = m.A * x + m.B * u;
    }
    EXPECT_LT(x.norm(), 1e-3);
}

TEST(TestLinearMPC, TimeVaryingModelOfOneModelIsTheSame)
{
    const auto m = model();
    const Matrix Q = Matrix::Identity(2, 2);
    const Matrix R = Matrix::Identity(1, 1);
    const Matrix P = bitsim::control::solve_dare(m.A, m.B, Q, R);
    Vector x0(2);
    x0 << -1.0, 0.3;

    LinearMPC invariant(2, 1, 15, tight());
    invariant.set_model(m.A, m.B);
    invariant.set_weights(Q, R, P);

    LinearMPC varying(2, 1, 15, tight());
    varying.set_model(std::vector<Matrix>(15, m.A), std::vector<Matrix>(15, m.B));
    varying.set_weights(Q, R, P);

    EXPECT_NEAR(invariant.solve(x0)[0], varying.solve(x0)[0], 1e-6);
}

TEST(TestLinearMPC, AffineTermAndReference)
{
    // the double integrator with a constant push c that acts like an input of 0.2: to hold
    // x = 1 at rest the input must be -0.2
    const auto m = model();
    const Vector c = m.B * 0.2;
    LinearMPC mpc(2, 1, 20, tight());
    mpc.set_model(std::vector<Matrix>(20, m.A), std::vector<Matrix>(20, m.B),
                  std::vector<Vector>(20, c));
    mpc.set_weights(Matrix::Identity(2, 2), 0.1 * Matrix::Identity(1, 1),
                    bitsim::control::solve_dare(m.A, m.B, Matrix::Identity(2, 2),
                                                0.1 * Matrix::Identity(1, 1)));
    Vector x_ref(2);
    x_ref << 1.0, 0.0;
    const Vector u_hold = Vector::Constant(1, -0.2);
    mpc.set_reference(x_ref, u_hold);

    Vector x = Vector::Zero(2);
    for (int k = 0; k < 300; ++k)
    {
        x = m.A * x + m.B * mpc.solve(x) + c;
    }
    EXPECT_NEAR(x[0], 1.0, 1e-3);
    EXPECT_NEAR(x[1], 0.0, 1e-3);
}

TEST(TestLinearMPC, BadInput)
{
    LinearMPC mpc(2, 1, 5);
    EXPECT_THROW(mpc.solve(Vector::Zero(2)), std::logic_error);
    EXPECT_THROW(mpc.set_model(Matrix::Identity(3, 3), Matrix::Zero(3, 1)), std::invalid_argument);
    EXPECT_THROW(mpc.set_input_bounds(Vector::Ones(1), Vector::Zero(1)), std::invalid_argument);
    EXPECT_THROW(LinearMPC(2, 1, 0), std::invalid_argument);
}

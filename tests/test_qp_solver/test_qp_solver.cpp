// The ADMM QP solver on problems with known solutions
#include "bitsim/maths/optimization/qp_solver.h"

#include <gtest/gtest.h>

#include <limits>

namespace
{
using bitsim::maths::optimization::QPSettings;
using bitsim::maths::optimization::QPSolver;
using bitsim::maths::optimization::QPStatus;
using Matrix = QPSolver::Matrix;
using Vector = QPSolver::Vector;

constexpr double INF = std::numeric_limits<double>::infinity();

// tolerances tight enough to compare the solutions with the exact ones
QPSettings tight()
{
    QPSettings settings;
    settings.eps_abs = 1e-9;
    settings.eps_rel = 1e-9;
    settings.max_iterations = 20000;
    return settings;
}
} // namespace

TEST(TestQPSolver, Unconstrained)
{
    Matrix P(2, 2);
    P << 2.0, 0.5, 0.5, 1.0;
    Vector q(2);
    q << -1.0, 2.0;

    QPSolver solver(tight());
    solver.setup(P, q, Matrix(0, 2), Vector(0), Vector(0));
    EXPECT_EQ(solver.solve(), QPStatus::SOLVED);
    const Vector expected = -P.ldlt().solve(q);
    EXPECT_LT((solver.solution() - expected).norm(), 1e-5);
}

TEST(TestQPSolver, ActiveBoxBounds)
{
    // min (x1 - 2)^2 + (x2 + 3)^2 with -1 <= x <= 1: the corner (1, -1)
    const Matrix P = 2.0 * Matrix::Identity(2, 2);
    Vector q(2);
    q << -4.0, 6.0;

    QPSolver solver(tight());
    solver.setup(P, q, Matrix::Identity(2, 2), Vector::Constant(2, -1.0), Vector::Constant(2, 1.0));
    EXPECT_EQ(solver.solve(), QPStatus::SOLVED);
    EXPECT_NEAR(solver.solution()[0], 1.0, 1e-4);
    EXPECT_NEAR(solver.solution()[1], -1.0, 1e-4);

    // the multiplier of an active upper bound is positive, of a lower one negative
    EXPECT_GT(solver.duals()[0], 0.0);
    EXPECT_LT(solver.duals()[1], 0.0);
}

TEST(TestQPSolver, Equality)
{
    // min x1^2 + x2^2 with x1 + x2 = 1
    QPSolver solver(tight());
    solver.setup(2.0 * Matrix::Identity(2, 2), Vector::Zero(2), Matrix::Ones(1, 2), Vector::Ones(1),
                 Vector::Ones(1));
    EXPECT_EQ(solver.solve(), QPStatus::SOLVED);
    EXPECT_NEAR(solver.solution()[0], 0.5, 1e-4);
    EXPECT_NEAR(solver.solution()[1], 0.5, 1e-4);
}

TEST(TestQPSolver, TheOSQPDemo)
{
    // the problem of OSQP's documentation, whose solution is (0.3, 0.7)
    Matrix P(2, 2);
    P << 4.0, 1.0, 1.0, 2.0;
    Vector q(2);
    q << 1.0, 1.0;
    Matrix C(3, 2);
    C << 1.0, 1.0, 1.0, 0.0, 0.0, 1.0;
    Vector l(3);
    l << 1.0, 0.0, 0.0;
    Vector u(3);
    u << 1.0, 0.7, 0.7;

    QPSolver solver(tight());
    solver.setup(P, q, C, l, u);
    EXPECT_EQ(solver.solve(), QPStatus::SOLVED);
    EXPECT_NEAR(solver.solution()[0], 0.3, 1e-4);
    EXPECT_NEAR(solver.solution()[1], 0.7, 1e-4);

    // solved again from its own solution, it is done at the first check
    const auto first = solver.iterations();
    EXPECT_EQ(solver.solve(), QPStatus::SOLVED);
    EXPECT_LT(solver.iterations(), first);
}

TEST(TestQPSolver, InfiniteBoundsAreNoBounds)
{
    QPSolver solver(tight());
    Vector l(2);
    l << -INF, 0.5;
    Vector u(2);
    u << INF, INF;
    solver.setup(2.0 * Matrix::Identity(2, 2), Vector::Zero(2), Matrix::Identity(2, 2), l, u);
    EXPECT_EQ(solver.solve(), QPStatus::SOLVED);
    EXPECT_NEAR(solver.solution()[0], 0.0, 1e-4);
    EXPECT_NEAR(solver.solution()[1], 0.5, 1e-4);
}

TEST(TestQPSolver, UpdateKeepsTheFactorisation)
{
    QPSolver solver(tight());
    solver.setup(2.0 * Matrix::Identity(1, 1), Vector::Zero(1), Matrix::Identity(1, 1),
                 Vector::Constant(1, -1.0), Vector::Constant(1, 1.0));
    solver.update(Vector::Constant(1, -10.0), Vector::Constant(1, -1.0), Vector::Constant(1, 2.0));
    EXPECT_EQ(solver.solve(), QPStatus::SOLVED);
    EXPECT_NEAR(solver.solution()[0], 2.0, 1e-4);

    // a row that turns into an equality changes its rho, and the solver refactorises
    solver.update(Vector::Zero(1), Vector::Constant(1, 0.25), Vector::Constant(1, 0.25));
    EXPECT_EQ(solver.solve(), QPStatus::SOLVED);
    EXPECT_NEAR(solver.solution()[0], 0.25, 1e-4);
}

TEST(TestQPSolver, BadInput)
{
    QPSolver solver(tight());
    EXPECT_EQ(solver.solve(), QPStatus::NOT_SET_UP);
    EXPECT_THROW(
        solver.setup(Matrix::Identity(2, 2), Vector::Zero(3), Matrix(0, 2), Vector(0), Vector(0)),
        std::invalid_argument);
    EXPECT_THROW(solver.setup(Matrix::Identity(1, 1), Vector::Zero(1), Matrix::Identity(1, 1),
                              Vector::Ones(1), Vector::Zero(1)),
                 std::invalid_argument);
}

// The Riccati solvers, the discretisation and the LQR controller, against cases with known
// answers
#include "bitsim/control/lqr.h"

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

namespace
{
using bitsim::control::Matrix;
using bitsim::control::Vector;

// the double integrator: position and velocity, driven by an acceleration
Matrix double_integrator_A() { return (Matrix(2, 2) << 0.0, 1.0, 0.0, 0.0).finished(); }
Matrix double_integrator_B() { return (Matrix(2, 1) << 0.0, 1.0).finished(); }

// the largest modulus of the eigenvalues
double spectral_radius(const Matrix &M) { return M.eigenvalues().cwiseAbs().maxCoeff(); }
} // namespace

TEST(TestLQR, DiscretiseTheDoubleIntegrator)
{
    const double dt = 0.1;
    const auto model =
        bitsim::control::discretise(double_integrator_A(), double_integrator_B(), dt);

    // the exact discrete model: x' = x + v dt + a dt^2 / 2, v' = v + a dt
    EXPECT_NEAR(model.A(0, 0), 1.0, 1e-12);
    EXPECT_NEAR(model.A(0, 1), dt, 1e-12);
    EXPECT_NEAR(model.A(1, 0), 0.0, 1e-12);
    EXPECT_NEAR(model.A(1, 1), 1.0, 1e-12);
    EXPECT_NEAR(model.B(0, 0), 0.5 * dt * dt, 1e-12);
    EXPECT_NEAR(model.B(1, 0), dt, 1e-12);
}

TEST(TestLQR, ScalarDARE)
{
    // p = a^2 p - a^2 b^2 p^2 / (r + b^2 p) + q with a = 2, b = q = r = 1: p^2 - 4 p - 1 = 0
    const Matrix A = Matrix::Constant(1, 1, 2.0);
    const Matrix B = Matrix::Constant(1, 1, 1.0);
    const Matrix Q = Matrix::Constant(1, 1, 1.0);
    const Matrix R = Matrix::Constant(1, 1, 1.0);
    const Matrix P = bitsim::control::solve_dare(A, B, Q, R);
    EXPECT_NEAR(P(0, 0), 2.0 + std::sqrt(5.0), 1e-10);

    // the gain stabilises the unstable system
    const auto solution = bitsim::control::dlqr(A, B, Q, R);
    EXPECT_LT(std::abs(2.0 - solution.K(0, 0)), 1.0);
}

TEST(TestLQR, ContinuousDoubleIntegrator)
{
    // with Q = I and R = 1 the gain is known: K = [1, sqrt(3)], P = [sqrt(3) 1; 1 sqrt(3)]
    const auto solution = bitsim::control::lqr(double_integrator_A(), double_integrator_B(),
                                               Matrix::Identity(2, 2), Matrix::Identity(1, 1));
    EXPECT_NEAR(solution.K(0, 0), 1.0, 1e-8);
    EXPECT_NEAR(solution.K(0, 1), std::sqrt(3.0), 1e-8);
    EXPECT_NEAR(solution.P(0, 0), std::sqrt(3.0), 1e-8);
    EXPECT_NEAR(solution.P(0, 1), 1.0, 1e-8);
    EXPECT_NEAR(solution.P(1, 1), std::sqrt(3.0), 1e-8);

    // the closed loop is stable: its eigenvalues are in the left half plane
    const Matrix closed = double_integrator_A() - double_integrator_B() * solution.K;
    EXPECT_LT(closed.eigenvalues().real().maxCoeff(), 0.0);
}

TEST(TestLQR, DiscreteRiccatiResidualOfALargerSystem)
{
    // an unstable system with 4 states and 2 inputs
    Matrix A(4, 4);
    A << 1.1, 0.2, 0.0, 0.1, 0.0, 0.9, 0.3, 0.0, 0.1, 0.0, 1.2, 0.2, 0.0, 0.1, 0.0, 0.8;
    Matrix B(4, 2);
    B << 1.0, 0.0, 0.0, 0.5, 0.2, 1.0, 0.0, 0.3;
    const Matrix Q = Matrix::Identity(4, 4);
    const Matrix R = 0.1 * Matrix::Identity(2, 2);

    const auto solution = bitsim::control::dlqr(A, B, Q, R);
    const Matrix &P = solution.P;
    const Matrix residual =
        A.transpose() * P * A -
        A.transpose() * P * B * (R + B.transpose() * P * B).inverse() * B.transpose() * P * A + Q -
        P;
    EXPECT_LT(residual.norm(), 1e-8);
    EXPECT_LT(spectral_radius(A - B * solution.K), 1.0);

    // the continuous one, for the same matrices taken as continuous
    const auto continuous = bitsim::control::lqr(A, B, Q, R);
    const Matrix &Pc = continuous.P;
    const Matrix residual_c =
        A.transpose() * Pc + Pc * A - Pc * B * R.inverse() * B.transpose() * Pc + Q;
    EXPECT_LT(residual_c.norm(), 1e-8);
    EXPECT_LT((A - B * continuous.K).eigenvalues().real().maxCoeff(), 0.0);
}

TEST(TestLQR, NoStabilisingSolution)
{
    // an unstable state that the input does not reach
    const Matrix A = Matrix::Constant(1, 1, 2.0);
    const Matrix B = Matrix::Zero(1, 1);
    const Matrix I = Matrix::Identity(1, 1);
    EXPECT_THROW(bitsim::control::solve_dare(A, B, I, I), std::runtime_error);
    EXPECT_THROW(bitsim::control::solve_care(A, B, I, I), std::runtime_error);
}

TEST(TestLQR, WrongSizes)
{
    const Matrix I2 = Matrix::Identity(2, 2);
    EXPECT_THROW(bitsim::control::solve_dare(I2, Matrix::Zero(3, 1), I2, Matrix::Identity(1, 1)),
                 std::invalid_argument);
    EXPECT_THROW(bitsim::control::discretise(I2, Matrix::Zero(2, 1), 0.0), std::invalid_argument);
}

TEST(TestLQR, ControllerAboutAnOperatingPoint)
{
    Matrix K(1, 2);
    K << 2.0, 3.0;
    bitsim::control::LQRController controller(K);

    Vector x_ref(2);
    x_ref << 1.0, 0.0;
    Vector u_ref(1);
    u_ref << 0.5;
    controller.set_reference(x_ref, u_ref);

    // u = u_ref - K (x - x_ref)
    Vector x(2);
    x << 1.5, -1.0;
    EXPECT_NEAR(controller.control(x)[0], 0.5 - (2.0 * 0.5 + 3.0 * -1.0), 1e-12);

    controller.set_input_bounds(Vector::Constant(1, -1.0), Vector::Constant(1, 1.0));
    EXPECT_NEAR(controller.control(x)[0], 1.0, 1e-12);
    EXPECT_THROW(controller.control(Vector::Zero(3)), std::invalid_argument);
}

TEST(TestLQR, ClosedLoopOfTheDiscretisedDoubleIntegrator)
{
    const auto model =
        bitsim::control::discretise(double_integrator_A(), double_integrator_B(), 0.05);
    const auto solution = bitsim::control::dlqr(model.A, model.B, Matrix::Identity(2, 2),
                                                0.1 * Matrix::Identity(1, 1));
    bitsim::control::LQRController controller(solution.K);

    // from 1 m away and at rest, it brings the state to the origin
    Vector x(2);
    x << 1.0, 0.0;
    for (int k = 0; k < 400; ++k)
    {
        x = model.A * x + model.B * controller.control(x);
    }
    EXPECT_LT(x.norm(), 1e-6);
}

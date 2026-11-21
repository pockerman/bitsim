#include "bitsim/control/lqr.h"

#include <unsupported/Eigen/MatrixFunctions>

#include <cmath>
#include <stdexcept>
#include <string>

namespace bitsim
{
namespace control
{
namespace
{

// The largest relative residual of a Riccati equation that is accepted
constexpr real_t RESIDUAL_TOLERANCE = 1.0e-6;
constexpr uint_t MAX_ITERATIONS = 100;

void check_sizes(const Matrix &A, const Matrix &B, const Matrix &Q, const Matrix &R,
                 const std::string &who)
{
    const auto n = A.rows();
    const auto m = B.cols();
    if (A.cols() != n || B.rows() != n || Q.rows() != n || Q.cols() != n || R.rows() != m ||
        R.cols() != m || n == 0 || m == 0)
    {
        throw std::invalid_argument(who + ": A must be n x n, B n x m, Q n x n and R m x m");
    }
}

Matrix symmetric(const Matrix &M) { return 0.5 * (M + M.transpose()); }

real_t relative(const Matrix &residual, const Matrix &P)
{
    return residual.norm() / std::max(1.0, P.norm());
}

} // namespace

DiscreteLinearModel discretise(const Matrix &A, const Matrix &B, real_t dt)
{
    const auto n = A.rows();
    const auto m = B.cols();
    if (A.cols() != n || B.rows() != n || n == 0 || m == 0)
    {
        throw std::invalid_argument("discretise: A must be n x n and B n x m");
    }
    if (!(dt > 0.0))
    {
        throw std::invalid_argument("discretise: dt must be positive");
    }

    // exp([A B; 0 0] dt) = [Ad Bd; 0 I]
    Matrix M = Matrix::Zero(n + m, n + m);
    M.topLeftCorner(n, n) = A * dt;
    M.topRightCorner(n, m) = B * dt;
    const Matrix E = M.exp();
    return {E.topLeftCorner(n, n), E.topRightCorner(n, m)};
}

Matrix solve_dare(const Matrix &A, const Matrix &B, const Matrix &Q, const Matrix &R)
{
    check_sizes(A, B, Q, R, "solve_dare");
    const auto n = A.rows();
    const Matrix I = Matrix::Identity(n, n);

    // The structure-preserving doubling algorithm: H_k converges to P
    Matrix Ak = A;
    Matrix Gk = symmetric(B * R.llt().solve(B.transpose()));
    Matrix Hk = symmetric(Q);
    for (uint_t k = 0; k < MAX_ITERATIONS; ++k)
    {
        const Eigen::PartialPivLU<Matrix> W(I + Gk * Hk);
        const Matrix WA = W.solve(Ak);
        const Matrix WG = W.solve(Gk);
        const Matrix H_next = symmetric(Hk + Ak.transpose() * Hk * WA);
        Gk = symmetric(Gk + Ak * WG * Ak.transpose());
        Ak = Ak * WA;

        const real_t change = (H_next - Hk).norm() / std::max(1.0, H_next.norm());
        Hk = H_next;
        if (change < 1.0e-13)
        {
            break;
        }
    }

    const Matrix &P = Hk;
    const Matrix BtP = B.transpose() * P;
    const Matrix residual =
        A.transpose() * P * A - A.transpose() * P * B * (R + BtP * B).ldlt().solve(BtP * A) + Q - P;
    if (!P.allFinite() || relative(residual, P) > RESIDUAL_TOLERANCE)
    {
        throw std::runtime_error("solve_dare: no stabilising solution; is (A, B) stabilisable "
                                 "and (A, Q) detectable?");
    }
    return P;
}

Matrix solve_care(const Matrix &A, const Matrix &B, const Matrix &Q, const Matrix &R)
{
    check_sizes(A, B, Q, R, "solve_care");
    const auto n = A.rows();

    // The Hamiltonian matrix [A, -B R^-1 B^T; -Q, -A^T]
    Matrix Z(2 * n, 2 * n);
    Z.topLeftCorner(n, n) = A;
    Z.topRightCorner(n, n) = -B * R.llt().solve(B.transpose());
    Z.bottomLeftCorner(n, n) = -Q;
    Z.bottomRightCorner(n, n) = -A.transpose();

    // The matrix sign function, by Newton's iteration with determinant scaling
    bool converged = false;
    for (uint_t k = 0; k < MAX_ITERATIONS; ++k)
    {
        const Eigen::PartialPivLU<Matrix> lu(Z);
        const real_t log_det = lu.matrixLU().diagonal().array().abs().log().sum();
        const real_t c = std::exp(-log_det / static_cast<real_t>(2 * n));
        const Matrix Z_next = 0.5 * (c * Z + lu.inverse() / c);
        const real_t change = (Z_next - Z).norm() / std::max(1.0, Z_next.norm());
        Z = Z_next;
        if (!Z.allFinite())
        {
            break;
        }
        if (change < 1.0e-12)
        {
            converged = true;
            break;
        }
    }

    // sign(H) is -1 on the stable subspace, spanned by [I; P]: (W + I) [I; P] = 0, so
    // [W12; W22 + I] P = -[W11 + I; W21]
    Matrix lhs(2 * n, n);
    lhs.topRows(n) = Z.topRightCorner(n, n);
    lhs.bottomRows(n) = Z.bottomRightCorner(n, n) + Matrix::Identity(n, n);
    Matrix rhs(2 * n, n);
    rhs.topRows(n) = -(Z.topLeftCorner(n, n) + Matrix::Identity(n, n));
    rhs.bottomRows(n) = -Z.bottomLeftCorner(n, n);
    const Matrix P = symmetric(lhs.colPivHouseholderQr().solve(rhs));

    const Matrix residual =
        A.transpose() * P + P * A - P * B * R.llt().solve(B.transpose() * P) + Q;
    if (!converged || !P.allFinite() || relative(residual, P) > RESIDUAL_TOLERANCE)
    {
        throw std::runtime_error("solve_care: no stabilising solution; is (A, B) stabilisable "
                                 "and (A, Q) detectable?");
    }
    return P;
}

LQRSolution dlqr(const Matrix &A, const Matrix &B, const Matrix &Q, const Matrix &R)
{
    const Matrix P = solve_dare(A, B, Q, R);
    const Matrix BtP = B.transpose() * P;
    return {(R + BtP * B).ldlt().solve(BtP * A), P};
}

LQRSolution lqr(const Matrix &A, const Matrix &B, const Matrix &Q, const Matrix &R)
{
    const Matrix P = solve_care(A, B, Q, R);
    return {R.llt().solve(B.transpose() * P), P};
}

LQRController::LQRController(const Matrix &K)
    : K_(K), x_ref_(Vector::Zero(K.cols())), u_ref_(Vector::Zero(K.rows()))
{
    if (K.rows() == 0 || K.cols() == 0)
    {
        throw std::invalid_argument("LQRController: the gain is empty");
    }
}

void LQRController::set_reference(const Vector &x_ref, const Vector &u_ref)
{
    if (x_ref.size() != K_.cols() || u_ref.size() != K_.rows())
    {
        throw std::invalid_argument("LQRController::set_reference: x_ref must have a value per "
                                    "state and u_ref one per input");
    }
    x_ref_ = x_ref;
    u_ref_ = u_ref;
}

void LQRController::set_input_bounds(const Vector &lower, const Vector &upper)
{
    if (lower.size() != K_.rows() || upper.size() != K_.rows() ||
        (lower.array() > upper.array()).any())
    {
        throw std::invalid_argument("LQRController::set_input_bounds: a lower and an upper bound "
                                    "per input, lower <= upper");
    }
    lower_ = lower;
    upper_ = upper;
    bounded_ = true;
}

Vector LQRController::control(const Vector &x) const
{
    if (x.size() != K_.cols())
    {
        throw std::invalid_argument("LQRController::control: x must have a value per state");
    }
    Vector u = u_ref_ - K_ * (x - x_ref_);
    if (bounded_)
    {
        u = u.cwiseMax(lower_).cwiseMin(upper_);
    }
    return u;
}

} // namespace control
} // namespace bitsim

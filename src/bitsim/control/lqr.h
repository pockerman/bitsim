#ifndef BITSIM_CONTROL_LQR_H
#define BITSIM_CONTROL_LQR_H

/**
 * @file lqr.h
 * @brief Linear quadratic regulators: the Riccati equations, the discretisation of a linear
 * model, and the state feedback about an operating point.
 */

#include "bitsim/bitsim_types.h"

#include <Eigen/Dense>

namespace bitsim
{
namespace control
{

/// @brief The matrices of the linear models and the controllers
using Matrix = RealMat;

/// @brief The vectors of the linear models and the controllers: states, inputs
using Vector = RealColVec;

///
/// @brief A linear model in discrete time, \f$x_{k+1} = A x_k + B u_k\f$
///
struct DiscreteLinearModel
{
    Matrix A;
    Matrix B;
};

///
/// @brief Discretise \f$\dot x = A x + B u\f$ for a step dt, with the input held over the step
/// (zero-order hold): \f$A_d = e^{A \Delta t}\f$ and \f$B_d = \int_0^{\Delta t} e^{A s} ds \, B\f$,
/// both from the exponential of one block matrix. Exact for a linear model.
///
DiscreteLinearModel discretise(const Matrix &A, const Matrix &B, real_t dt);

///
/// @brief Solve the discrete algebraic Riccati equation
/// \f[
/// P = A^T P A - A^T P B (R + B^T P B)^{-1} B^T P A + Q
/// \f]
/// for its stabilising solution, with the structure-preserving doubling algorithm, which
/// converges quadratically. (A, B) must be stabilisable, (A, Q) detectable, Q symmetric and
/// positive semi-definite and R symmetric and positive definite.
/// @throws std::invalid_argument if the sizes do not match
/// @throws std::runtime_error if it does not converge, which happens when there is no
/// stabilising solution
///
Matrix solve_dare(const Matrix &A, const Matrix &B, const Matrix &Q, const Matrix &R);

///
/// @brief Solve the continuous algebraic Riccati equation
/// \f[
/// A^T P + P A - P B R^{-1} B^T P + Q = 0
/// \f]
/// for its stabilising solution, from the matrix sign function of the Hamiltonian matrix: its
/// stable invariant subspace is spanned by \f$[I; P]\f$. The same conditions as solve_dare().
/// @throws std::invalid_argument if the sizes do not match
/// @throws std::runtime_error if it does not converge, which happens when there is no
/// stabilising solution
///
Matrix solve_care(const Matrix &A, const Matrix &B, const Matrix &Q, const Matrix &R);

///
/// @brief The gain of a linear quadratic regulator and the cost matrix it comes from: the input
/// \f$u = -K x\f$ minimises the cost \f$\sum x^T Q x + u^T R u\f$, or its integral in continuous
/// time, and the cost from x on is \f$x^T P x\f$
///
struct LQRSolution
{
    Matrix K;
    Matrix P;
};

///
/// @brief The LQR of \f$x_{k+1} = A x_k + B u_k\f$: \f$K = (R + B^T P B)^{-1} B^T P A\f$ with P
/// from solve_dare()
///
LQRSolution dlqr(const Matrix &A, const Matrix &B, const Matrix &Q, const Matrix &R);

///
/// @brief The LQR of \f$\dot x = A x + B u\f$: \f$K = R^{-1} B^T P\f$ with P from solve_care()
///
LQRSolution lqr(const Matrix &A, const Matrix &B, const Matrix &Q, const Matrix &R);

/**
 * @class LQRController
 * @ingroup bitsim_control
 * @brief A linear state feedback about an operating point,
 * \f$u = u_{ref} - K (x - x_{ref})\f$, with optional bounds on each input.
 *
 * The gain usually comes from lqr() or dlqr() for a model linearised about the operating point,
 * such as a hover or a trim, and the reference is that point: \f$x_{ref}\f$ the state and
 * \f$u_{ref}\f$ the input that holds it there. Changing \f$x_{ref}\f$ to another state the same
 * input holds, such as another position at hover, makes the controller go there.
 *
 * With bounds, each input is clipped to them. The gain was designed without them, so a clipped
 * input no longer minimises the cost; LinearMPC takes the bounds into account instead.
 */
class LQRController
{
  public:
    ///
    /// @param K The gain, inputs x states
    ///
    explicit LQRController(const Matrix &K);

    ///
    /// @brief Set the operating point: the state to hold and the input that holds it. Both start
    /// at zero
    /// @throws std::invalid_argument if the sizes do not match the gain
    ///
    void set_reference(const Vector &x_ref, const Vector &u_ref);

    ///
    /// @brief Clip each input to [lower, upper]
    /// @throws std::invalid_argument if the sizes do not match, or lower > upper
    ///
    void set_input_bounds(const Vector &lower, const Vector &upper);

    ///
    /// @return The input for the state x
    ///
    Vector control(const Vector &x) const;

    const Matrix &gain() const noexcept { return K_; }
    const Vector &state_reference() const noexcept { return x_ref_; }
    const Vector &input_reference() const noexcept { return u_ref_; }
    uint_t n_states() const noexcept { return static_cast<uint_t>(K_.cols()); }
    uint_t n_inputs() const noexcept { return static_cast<uint_t>(K_.rows()); }

  private:
    Matrix K_;
    Vector x_ref_;
    Vector u_ref_;
    bool bounded_{false};
    Vector lower_;
    Vector upper_;
};

} // namespace control
} // namespace bitsim

#endif // BITSIM_CONTROL_LQR_H

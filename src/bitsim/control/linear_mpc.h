#ifndef BITSIM_CONTROL_LINEAR_MPC_H
#define BITSIM_CONTROL_LINEAR_MPC_H

#include "bitsim/bitsim_export.h"
#include "bitsim/bitsim_types.h"
#include "bitsim/control/lqr.h"
#include "bitsim/maths/optimization/qp_solver.h"

#include <vector>

namespace bitsim
{
namespace control
{

/**
 * @class LinearMPC
 * @ingroup bitsim_control
 * @brief Model predictive control with a linear model, a quadratic cost and bounds on the inputs
 * and the states.
 *
 * At every step it plans the inputs \f$u_0, \dots, u_{N-1}\f$ over a horizon of N steps, from the
 * state now, \f$x_0\f$, that minimise
 * \f[
 * \sum_{k=1}^{N-1} (x_k - r_k)^T Q (x_k - r_k) + (x_N - r_N)^T P (x_N - r_N)
 * + \sum_{k=0}^{N-1} (u_k - v_k)^T R (u_k - v_k)
 * \f]
 * subject to the model \f$x_{k+1} = A_k x_k + B_k u_k + c_k\f$ and the bounds
 * \f$u_{min} \le u_k \le u_{max}\f$ and \f$x_{min} \le x_k \le x_{max}\f$, hard or soft, with r the
 * reference of the states and v that of the inputs. The application uses \f$u_0\f$ and plans again
 * at the next step, from the state then.
 *
 * The model is time-invariant, set_model(A, B), or changes along the horizon,
 * set_model(As, Bs, cs): a nonlinear model linearised along a reference trajectory, with
 * \f$c_k = f(\bar x_k, \bar u_k) - A_k \bar x_k - B_k \bar u_k\f$. The terminal weight P is by
 * default the solution of the discrete Riccati equation of a time-invariant model, which makes
 * the cost beyond the horizon that of the LQR, and Q otherwise.
 *
 * The problem is condensed: the states are written in terms of \f$x_0\f$ and the inputs, so the
 * QP has the N m inputs as its variables. maths::optimization::QPSolver solves it, starting from
 * the solution of the step before. A bound of \f$\pm\infty\f$ is no bound. State bounds that the
 * system cannot meet make the QP infeasible: solve() then returns the inputs of the last
 * iteration, and status() says that it did not converge.
 */
class BITSIM_EXPORT LinearMPC
{
  public:
    using QPSettings = maths::optimization::QPSettings;
    using QPStatus = maths::optimization::QPStatus;

    ///
    /// @param n_states The number of states
    /// @param n_inputs The number of inputs
    /// @param horizon The number of steps it plans, N
    /// @param settings The settings of the QP solver
    ///
    LinearMPC(uint_t n_states, uint_t n_inputs, uint_t horizon,
              const QPSettings &settings = QPSettings());

    ///
    /// @brief A time-invariant model, \f$x_{k+1} = A x_k + B u_k\f$, in discrete time; see
    /// discretise()
    ///
    void set_model(const Matrix &A, const Matrix &B);

    ///
    /// @brief A model that changes along the horizon: N matrices A and B, and N vectors c, or
    /// none for c = 0
    ///
    void set_model(const std::vector<Matrix> &A, const std::vector<Matrix> &B,
                   const std::vector<Vector> &c = {});

    ///
    /// @brief The weights. Without P, it is the Riccati solution of a time-invariant model, or Q
    ///
    void set_weights(const Matrix &Q, const Matrix &R);
    void set_weights(const Matrix &Q, const Matrix &R, const Matrix &P);

    ///
    /// @brief Bounds on each input; \f$\pm\infty\f$ for none
    ///
    void set_input_bounds(const Vector &lower, const Vector &upper);

    ///
    /// @brief Bounds on each state, at the steps 1 to N.
    /// @param lower The lower bound of each state; \f$-\infty\f$ for none
    /// @param upper The upper bound of each state; \f$+\infty\f$ for none
    /// @param penalty 0 for hard bounds. A positive penalty makes them soft: the plan may break
    /// a bound by s, at the cost penalty \f$(s + s^2)\f$, which keeps the QP feasible when the
    /// system cannot keep to the bounds, as after a large disturbance. The linear part makes the
    /// penalty exact: when the bounds can be met and the penalty is large enough, the plan meets
    /// them. ADMM converges slowly when the penalty dwarfs the weights: 100 to 1000 times the
    /// largest weight of Q is enough to make the bounds hold, and converges well.
    ///
    void set_state_bounds(const Vector &lower, const Vector &upper, real_t penalty = 0.0);

    ///
    /// @brief The same reference at every step: the state and the input that holds it
    ///
    void set_reference(const Vector &x_ref, const Vector &u_ref);

    ///
    /// @brief A reference that changes along the horizon: the states at the steps 1 to N, N of
    /// them, and the inputs at the steps 0 to N - 1, N of them
    ///
    void set_reference(const std::vector<Vector> &x_ref, const std::vector<Vector> &u_ref);

    ///
    /// @brief Plan from the state x0
    /// @return The input to use now, \f$u_0\f$
    /// @throws std::logic_error if the model or the weights are not set
    ///
    Vector solve(const Vector &x0);

    ///
    /// @return The planned states, \f$x_0\f$ to \f$x_N\f$, of the last solve
    ///
    const std::vector<Vector> &predicted_states() const noexcept { return x_plan_; }

    ///
    /// @return The planned inputs, \f$u_0\f$ to \f$u_{N-1}\f$, of the last solve
    ///
    const std::vector<Vector> &predicted_inputs() const noexcept { return u_plan_; }

    QPStatus status() const noexcept { return solver_.status(); }
    uint_t iterations() const noexcept { return solver_.iterations(); }
    uint_t horizon() const noexcept { return N_; }
    uint_t n_states() const noexcept { return n_; }
    uint_t n_inputs() const noexcept { return m_; }
    const Matrix &terminal_weight() const noexcept { return P_; }

  private:
    void check_vector_(const Vector &v, uint_t size, const char *what) const;
    void build_structure_();
    void build_qp_(const Vector &x0);

    uint_t n_;
    uint_t m_;
    uint_t N_;
    maths::optimization::QPSolver solver_;

    std::vector<Matrix> A_;
    std::vector<Matrix> B_;
    std::vector<Vector> c_;
    bool time_invariant_{true};
    bool has_model_{false};

    Matrix Q_;
    Matrix R_;
    Matrix P_;
    bool has_weights_{false};
    bool user_terminal_{false};

    Vector u_min_;
    Vector u_max_;
    Vector x_min_;
    Vector x_max_;
    real_t state_penalty_{0.0};

    std::vector<Vector> x_ref_;
    std::vector<Vector> u_ref_;

    std::vector<index_t> state_rows_; // the states x_1..x_N with a finite bound
    bool structure_dirty_{true};
    bool qp_dirty_{true};

    std::vector<Vector> x_plan_;
    std::vector<Vector> u_plan_;
};

} // namespace control
} // namespace bitsim

#endif // BITSIM_CONTROL_LINEAR_MPC_H

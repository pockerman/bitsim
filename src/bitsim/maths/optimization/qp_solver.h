#ifndef BITSIM_MATHS_OPTIMIZATION_QP_SOLVER_H
#define BITSIM_MATHS_OPTIMIZATION_QP_SOLVER_H

#include "bitsim/bitsim_export.h"
#include "bitsim/bitsim_types.h"

#include <Eigen/Dense>

#include <memory>
#include <vector>

namespace bitsim
{
namespace maths
{
namespace optimization
{

///
/// @brief The settings of QPSolver
///
struct QPSettings
{
    real_t rho{0.1};               ///< the step of the dual update; equality rows use 1000 rho
    real_t sigma{1.0e-6};          ///< a small regularisation of the x update
    real_t alpha{1.6};             ///< over-relaxation, in (0, 2)
    real_t eps_abs{1.0e-4};        ///< absolute tolerance of the residuals
    real_t eps_rel{1.0e-4};        ///< relative tolerance of the residuals
    uint_t max_iterations{4000};   ///< iterations at most
    uint_t check_every{10};        ///< iterations between checks of the residuals
    bool adaptive_rho{true};       ///< change rho to balance the residuals, as OSQP does
    uint_t scaling_iterations{10}; ///< iterations of the Ruiz equilibration; 0 for none
};

///
/// @brief A non-zero of a sparse matrix: its row, its column and its value
///
struct QPEntry
{
    index_t row;
    index_t col;
    real_t value;
};

///
/// @brief How a solve ended
///
enum class QPStatus
{
    SOLVED,         ///< the residuals are within the tolerances
    MAX_ITERATIONS, ///< stopped at max_iterations; the solution is the last iterate
    NOT_SET_UP      ///< solve() before setup()
};

/**
 * @class QPSolver
 * @ingroup bitsim_control
 * @brief A solver of the convex quadratic program
 * \f[
 * \min_x \tfrac{1}{2} x^T P x + q^T x \quad \text{subject to} \quad l \le C x \le u ,
 * \f]
 * with P symmetric positive semi-definite, by the alternating direction method of multipliers
 * (ADMM) of OSQP (Stellato et al., OSQP: an operator splitting solver for quadratic programs,
 * 2020). A row with \f$l_i = u_i\f$ is an equality, and a bound of \f$\pm\infty\f$ is no bound.
 *
 * Every iteration solves one linear system with the matrix \f$P + \sigma I + C^T \rho C\f$,
 * factorised with a sparse Cholesky factorisation, so an iteration costs two sparse triangular
 * solves. As in OSQP, the problem is first equilibrated: the variables, the constraints and the
 * cost are scaled so that the rows and the columns of its matrices have similar norms, which ADMM
 * needs to converge in reasonable time when the numbers of the problem span orders of magnitude.
 * And \f$\rho\f$ adapts to balance the primal and the dual residuals; that needs a new
 * factorisation, so it changes only when it is off by a factor of 5. The tolerances apply to the
 * problem as it was given.
 *
 * It suits the problems of model predictive control, with a few thousand variables at most. The
 * solver keeps its last solution, and the next solve starts from it, which in MPC, where one
 * problem is much like the last, saves most of the iterations. ADMM converges to a modest
 * accuracy quickly and to a high one slowly: the default tolerances, 1e-4, are enough for
 * control, and much tighter ones can take thousands of iterations.
 */
class BITSIM_EXPORT QPSolver
{
  public:
    using Matrix = RealMat;
    using Vector = RealColVec;

    explicit QPSolver(const QPSettings &settings = QPSettings());
    ~QPSolver();
    QPSolver(const QPSolver &other);
    QPSolver &operator=(const QPSolver &other);
    QPSolver(QPSolver &&other) noexcept;
    QPSolver &operator=(QPSolver &&other) noexcept;

    ///
    /// @brief Set the matrices and the bounds, and factorise the matrix of the iterations
    /// @throws std::invalid_argument if the sizes do not match, or l > u somewhere
    /// @throws std::runtime_error if the matrix cannot be factorised, which means P is not
    /// positive semi-definite
    ///
    void setup(const Matrix &P, const Vector &q, const Matrix &C, const Vector &l, const Vector &u);

    ///
    /// @brief The same, with P, n x n, and C, m x n, given by their non-zeros: the solver works
    /// with sparse matrices inside, so a problem with mostly zeros, as in model predictive
    /// control, costs only its non-zeros. Entries at the same place are added up
    ///
    void setup(index_t n, const std::vector<QPEntry> &P, const Vector &q, index_t m,
               const std::vector<QPEntry> &C, const Vector &l, const Vector &u);

    ///
    /// @brief Change q and the bounds, keeping P and C and the factorisation
    ///
    void update(const Vector &q, const Vector &l, const Vector &u);

    ///
    /// @brief Solve, starting from the last solution
    ///
    QPStatus solve();

    ///
    /// @brief Start the next solve from x, and from no constraint being active
    ///
    void warm_start(const Vector &x);

    ///
    /// @brief Forget the last solution: the next solve starts from zero
    ///
    void reset();

    const Vector &solution() const noexcept { return x_out_; }

    ///
    /// @return The multipliers of the constraints: positive where an upper bound is active,
    /// negative where a lower one is
    ///
    const Vector &duals() const noexcept { return y_out_; }

    uint_t iterations() const noexcept { return iterations_; }
    real_t primal_residual() const noexcept { return primal_residual_; }
    real_t dual_residual() const noexcept { return dual_residual_; }
    QPStatus status() const noexcept { return status_; }
    const QPSettings &settings() const noexcept { return settings_; }

  private:
    // the sparse matrices and the factorisation, kept out of this header: Eigen's sparse
    // matrices must not be included before Chrono's headers, which extend them
    struct Sparse;

    void setup_(Sparse &problem, const Vector &q, const Vector &l, const Vector &u);
    void check_bounds_(const Vector &l, const Vector &u) const;
    bool factorise_();
    void equilibrate_(const Vector &q);
    void unscale_();
    void compute_residuals_();

    QPSettings settings_;
    std::unique_ptr<Sparse> sp_;
    Vector q_;
    Vector l_;
    Vector u_;
    Vector rho_;
    real_t rho_scalar_{0.1};
    bool is_set_up_{false};

    // the scaling: the variables by D, the constraints by E and the cost by cost_scale_
    Vector D_;
    Vector E_;
    real_t cost_scale_{1.0};

    // the iterates of the scaled problem, and the solution and the duals of the given one
    Vector x_;
    Vector z_;
    Vector y_;
    Vector x_out_;
    Vector y_out_;
    real_t primal_scale_{0.0};
    real_t dual_scale_{0.0};
    uint_t iterations_{0};
    real_t primal_residual_{0.0};
    real_t dual_residual_{0.0};
    QPStatus status_{QPStatus::NOT_SET_UP};
};

} // namespace optimization
} // namespace maths
} // namespace bitsim

#endif // BITSIM_MATHS_OPTIMIZATION_QP_SOLVER_H

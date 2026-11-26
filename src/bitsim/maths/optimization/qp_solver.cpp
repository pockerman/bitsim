#include "bitsim/maths/optimization/qp_solver.h"

#include <Eigen/Sparse>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace bitsim
{
namespace maths
{
namespace optimization
{
namespace
{
constexpr real_t RHO_MIN = 1.0e-6;
constexpr real_t RHO_MAX = 1.0e6;
constexpr real_t SCALE_MIN = 1.0e-4;
constexpr real_t SCALE_MAX = 1.0e4;

using SparseMatrix = Eigen::SparseMatrix<real_t>;
using Vector = QPSolver::Vector;

// The infinity norm of every column of a sparse matrix
Vector column_norms(const SparseMatrix &M)
{
    Vector norms = Vector::Zero(M.cols());
    for (index_t j = 0; j < M.outerSize(); ++j)
    {
        for (SparseMatrix::InnerIterator it(M, j); it; ++it)
        {
            norms[it.col()] = std::max(norms[it.col()], std::abs(it.value()));
        }
    }
    return norms;
}

// The scale that brings a norm to 1, within limits; a zero norm is left alone
real_t scale_for(real_t norm)
{
    return norm < SCALE_MIN ? 1.0 : std::clamp(1.0 / std::sqrt(norm), SCALE_MIN, SCALE_MAX);
}
} // namespace

struct QPSolver::Sparse
{
    SparseMatrix P;
    SparseMatrix C;
    SparseMatrix Ct;
    Eigen::SimplicialLDLT<SparseMatrix> kkt;
};

QPSolver::QPSolver(const QPSettings &settings)
    : settings_(settings), sp_(std::make_unique<Sparse>())
{
}

QPSolver::~QPSolver() = default;

QPSolver::QPSolver(const QPSolver &other) : sp_(std::make_unique<Sparse>()) { *this = other; }

QPSolver &QPSolver::operator=(const QPSolver &other)
{
    if (this == &other)
    {
        return *this;
    }
    settings_ = other.settings_;
    sp_ = std::make_unique<Sparse>();
    sp_->P = other.sp_->P;
    sp_->C = other.sp_->C;
    sp_->Ct = other.sp_->Ct;
    q_ = other.q_;
    l_ = other.l_;
    u_ = other.u_;
    rho_ = other.rho_;
    rho_scalar_ = other.rho_scalar_;
    is_set_up_ = other.is_set_up_;
    D_ = other.D_;
    E_ = other.E_;
    cost_scale_ = other.cost_scale_;
    x_ = other.x_;
    z_ = other.z_;
    y_ = other.y_;
    x_out_ = other.x_out_;
    y_out_ = other.y_out_;
    primal_scale_ = other.primal_scale_;
    dual_scale_ = other.dual_scale_;
    iterations_ = other.iterations_;
    primal_residual_ = other.primal_residual_;
    dual_residual_ = other.dual_residual_;
    status_ = other.status_;
    // the factorisation cannot be copied: make it again
    if (is_set_up_ && !factorise_())
    {
        throw std::runtime_error("QPSolver: cannot factorise the copy");
    }
    return *this;
}

QPSolver::QPSolver(QPSolver &&other) noexcept = default;
QPSolver &QPSolver::operator=(QPSolver &&other) noexcept = default;

void QPSolver::check_bounds_(const Vector &l, const Vector &u) const
{
    if (l.size() != sp_->C.rows() || u.size() != sp_->C.rows())
    {
        throw std::invalid_argument("QPSolver: l and u must have a value per row of C");
    }
    if ((l.array() > u.array()).any())
    {
        throw std::invalid_argument("QPSolver: l must not be above u");
    }
}

void QPSolver::setup(const Matrix &P, const Vector &q, const Matrix &C, const Vector &l,
                     const Vector &u)
{
    if (P.rows() != P.cols() || C.cols() != P.rows())
    {
        throw std::invalid_argument("QPSolver::setup: P must be n x n, q of size n and C m x n");
    }
    Sparse problem;
    problem.P = P.sparseView();
    problem.C = C.sparseView();
    setup_(problem, q, l, u);
}

void QPSolver::setup(index_t n, const std::vector<QPEntry> &P, const Vector &q, index_t m,
                     const std::vector<QPEntry> &C, const Vector &l, const Vector &u)
{
    const auto to_triplets = [](const std::vector<QPEntry> &entries, index_t rows, index_t cols)
    {
        std::vector<Eigen::Triplet<real_t>> triplets;
        triplets.reserve(entries.size());
        for (const auto &e : entries)
        {
            if (e.row < 0 || e.row >= rows || e.col < 0 || e.col >= cols)
            {
                throw std::invalid_argument("QPSolver::setup: an entry is outside its matrix");
            }
            triplets.emplace_back(e.row, e.col, e.value);
        }
        return triplets;
    };
    Sparse problem;
    problem.P.resize(n, n);
    const auto p = to_triplets(P, n, n);
    problem.P.setFromTriplets(p.begin(), p.end());
    problem.C.resize(m, n);
    const auto c = to_triplets(C, m, n);
    problem.C.setFromTriplets(c.begin(), c.end());
    setup_(problem, q, l, u);
}

void QPSolver::setup_(Sparse &problem, const Vector &q, const Vector &l, const Vector &u)
{
    const SparseMatrix &P = problem.P;
    const SparseMatrix &C = problem.C;
    const auto n = P.rows();
    if (P.cols() != n || q.size() != n || C.cols() != n || n == 0)
    {
        throw std::invalid_argument("QPSolver::setup: P must be n x n, q of size n and C m x n");
    }
    if (!(settings_.rho > 0.0 && settings_.sigma > 0.0 && settings_.alpha > 0.0 &&
          settings_.alpha < 2.0))
    {
        throw std::invalid_argument("QPSolver::setup: rho and sigma must be positive and alpha "
                                    "in (0, 2)");
    }

    const bool same_size = is_set_up_ && x_.size() == n && z_.size() == C.rows();
    // keep the last solution, unscaled, to start from it with the new scaling
    const Vector x_last = same_size ? Vector(D_.cwiseProduct(x_)) : Vector();
    const Vector y_last = same_size ? Vector(E_.cwiseProduct(y_) / cost_scale_) : Vector();

    sp_->P = 0.5 * (SparseMatrix(P.transpose()) + P);
    sp_->C = C;
    if (l.size() != C.rows() || u.size() != C.rows() || (l.array() > u.array()).any())
    {
        throw std::invalid_argument("QPSolver: l and u must have a value per row of C, with l "
                                    "not above u");
    }
    equilibrate_(q);
    sp_->Ct = sp_->C.transpose();
    q_ = cost_scale_ * D_.cwiseProduct(q);
    l_ = E_.cwiseProduct(l);
    u_ = E_.cwiseProduct(u);

    rho_scalar_ = settings_.rho;
    if (!factorise_())
    {
        is_set_up_ = false;
        throw std::runtime_error("QPSolver::setup: cannot factorise; is P positive "
                                 "semi-definite?");
    }

    if (same_size)
    {
        x_ = x_last.cwiseQuotient(D_);
        y_ = cost_scale_ * y_last.cwiseQuotient(E_);
        z_ = (sp_->C * x_).cwiseMax(l_).cwiseMin(u_);
    }
    else
    {
        x_ = Vector::Zero(n);
        z_ = Vector::Zero(sp_->C.rows());
        y_ = Vector::Zero(sp_->C.rows());
    }
    is_set_up_ = true;
    status_ = QPStatus::MAX_ITERATIONS;
    unscale_();
}

void QPSolver::equilibrate_(const Vector &q)
{
    // Ruiz equilibration, as in OSQP: scale the variables by D and the constraints by E until
    // every column of the matrix [P C^T; C 0] has a norm near 1, then the cost by c. The scaled
    // problem has P' = c D P D, q' = c D q, C' = E C D and the bounds E l, E u
    const auto n = sp_->P.rows();
    const auto m = sp_->C.rows();
    D_ = Vector::Ones(n);
    E_ = Vector::Ones(m);
    cost_scale_ = 1.0;
    for (uint_t k = 0; k < settings_.scaling_iterations; ++k)
    {
        const Vector p_norms = column_norms(sp_->P);
        const Vector c_norms = column_norms(sp_->C);
        const Vector row_norms = m > 0 ? column_norms(SparseMatrix(sp_->C.transpose())) : Vector();
        Vector d(n);
        for (index_t j = 0; j < n; ++j)
        {
            d[j] = scale_for(std::max(p_norms[j], m > 0 ? c_norms[j] : 0.0));
        }
        Vector e(m);
        for (index_t i = 0; i < m; ++i)
        {
            e[i] = scale_for(row_norms[i]);
        }
        sp_->P = d.asDiagonal() * sp_->P * d.asDiagonal();
        sp_->C = e.asDiagonal() * sp_->C * d.asDiagonal();
        D_ = D_.cwiseProduct(d);
        E_ = E_.cwiseProduct(e);
    }

    // the cost: bring the mean column norm of P, or the norm of q, to 1
    if (settings_.scaling_iterations > 0)
    {
        const real_t p_mean = n > 0 ? column_norms(sp_->P).mean() : 0.0;
        const real_t q_norm = D_.cwiseProduct(q).lpNorm<Eigen::Infinity>();
        const real_t norm = std::max(p_mean, q_norm);
        cost_scale_ = norm < SCALE_MIN ? 1.0 : std::clamp(1.0 / norm, SCALE_MIN, SCALE_MAX);
        sp_->P *= cost_scale_;
    }
}

void QPSolver::unscale_()
{
    x_out_ = D_.cwiseProduct(x_);
    y_out_ = E_.cwiseProduct(y_) / cost_scale_;
}

bool QPSolver::factorise_()
{
    // as in OSQP: equality rows get a larger rho, so that they hold sooner, and rows without
    // bounds a tiny one, since they constrain nothing
    rho_.resize(sp_->C.rows());
    for (index_t i = 0; i < sp_->C.rows(); ++i)
    {
        if (l_[i] == u_[i])
        {
            rho_[i] = 1.0e3 * rho_scalar_;
        }
        else if (std::isinf(l_[i]) && std::isinf(u_[i]))
        {
            rho_[i] = RHO_MIN;
        }
        else
        {
            rho_[i] = rho_scalar_;
        }
    }

    const auto n = sp_->P.rows();
    SparseMatrix I(n, n);
    I.setIdentity();
    const SparseMatrix K = sp_->P + settings_.sigma * I + sp_->Ct * rho_.asDiagonal() * sp_->C;
    sp_->kkt.compute(K);
    return sp_->kkt.info() == Eigen::Success;
}

void QPSolver::update(const Vector &q, const Vector &l, const Vector &u)
{
    if (!is_set_up_)
    {
        throw std::logic_error("QPSolver::update: call setup first");
    }
    if (q.size() != q_.size())
    {
        throw std::invalid_argument("QPSolver::update: q has the wrong size");
    }
    check_bounds_(l, u);
    const Vector ls = E_.cwiseProduct(l);
    const Vector us = E_.cwiseProduct(u);
    bool refactorise = false;
    for (index_t i = 0; i < l.size(); ++i)
    {
        // the rho of a row depends on whether it is an equality, and whether it is bounded
        refactorise =
            refactorise || (ls[i] == us[i]) != (l_[i] == u_[i]) ||
            (std::isinf(ls[i]) && std::isinf(us[i])) != (std::isinf(l_[i]) && std::isinf(u_[i]));
    }
    q_ = cost_scale_ * D_.cwiseProduct(q);
    l_ = ls;
    u_ = us;
    if (refactorise && !factorise_())
    {
        throw std::runtime_error("QPSolver::update: cannot factorise");
    }
}

void QPSolver::warm_start(const Vector &x)
{
    if (!is_set_up_ || x.size() != x_.size())
    {
        throw std::invalid_argument("QPSolver::warm_start: x has the wrong size");
    }
    x_ = x.cwiseQuotient(D_);
    z_ = (sp_->C * x_).cwiseMax(l_).cwiseMin(u_);
    y_.setZero();
    unscale_();
}

void QPSolver::reset()
{
    if (is_set_up_)
    {
        x_.setZero();
        z_.setZero();
        y_.setZero();
        unscale_();
    }
}

void QPSolver::compute_residuals_()
{
    // the residuals of the problem as it was given, not of the scaled one
    const Vector Cx = (sp_->C * x_).cwiseQuotient(E_);
    const Vector z = z_.cwiseQuotient(E_);
    const Vector Px = (sp_->P * x_).cwiseQuotient(D_) / cost_scale_;
    const Vector Cty = (sp_->Ct * y_).cwiseQuotient(D_) / cost_scale_;
    const Vector q = q_.cwiseQuotient(D_) / cost_scale_;

    const bool has_rows = sp_->C.rows() > 0;
    primal_residual_ = has_rows ? (Cx - z).lpNorm<Eigen::Infinity>() : 0.0;
    dual_residual_ = (Px + q + Cty).lpNorm<Eigen::Infinity>();
    primal_scale_ =
        has_rows ? std::max(Cx.lpNorm<Eigen::Infinity>(), z.lpNorm<Eigen::Infinity>()) : 0.0;
    dual_scale_ =
        std::max({Px.lpNorm<Eigen::Infinity>(), has_rows ? Cty.lpNorm<Eigen::Infinity>() : 0.0,
                  q.lpNorm<Eigen::Infinity>()});
}

QPStatus QPSolver::solve()
{
    if (!is_set_up_)
    {
        status_ = QPStatus::NOT_SET_UP;
        return status_;
    }

    const real_t sigma = settings_.sigma;
    const real_t alpha = settings_.alpha;
    const uint_t check_every = std::max<uint_t>(1, settings_.check_every);

    status_ = QPStatus::MAX_ITERATIONS;
    iterations_ = 0;
    for (uint_t k = 1; k <= settings_.max_iterations; ++k)
    {
        // the x and z updates: the solution of the equality constrained step
        const Vector rhs = sigma * x_ - q_ + sp_->Ct * (rho_.cwiseProduct(z_) - y_);
        const Vector x_tilde = sp_->kkt.solve(rhs);
        const Vector z_tilde = sp_->C * x_tilde;

        const Vector x_next = alpha * x_tilde + (1.0 - alpha) * x_;
        const Vector z_relaxed = alpha * z_tilde + (1.0 - alpha) * z_;
        const Vector z_next = (z_relaxed + y_.cwiseQuotient(rho_)).cwiseMax(l_).cwiseMin(u_);
        y_ += rho_.cwiseProduct(z_relaxed - z_next);
        x_ = x_next;
        z_ = z_next;
        iterations_ = k;

        if (k % check_every == 0 || k == settings_.max_iterations)
        {
            compute_residuals_();
            const real_t primal_scale = primal_scale_;
            const real_t dual_scale = dual_scale_;
            if (primal_residual_ <= settings_.eps_abs + settings_.eps_rel * primal_scale &&
                dual_residual_ <= settings_.eps_abs + settings_.eps_rel * dual_scale)
            {
                status_ = QPStatus::SOLVED;
                break;
            }

            // OSQP's rule: balance the two residuals, each relative to its scale. A new rho
            // needs a new factorisation, so rho only changes when it is off by a factor of 5
            if (settings_.adaptive_rho && sp_->C.rows() > 0)
            {
                const real_t primal = primal_residual_ / std::max(primal_scale, 1.0e-10);
                const real_t dual = dual_residual_ / std::max(dual_scale, 1.0e-10);
                const real_t rho_new = std::clamp(
                    rho_scalar_ * std::sqrt(primal / std::max(dual, 1.0e-10)), RHO_MIN, RHO_MAX);
                if (rho_new > 5.0 * rho_scalar_ || rho_new < 0.2 * rho_scalar_)
                {
                    rho_scalar_ = rho_new;
                    factorise_();
                }
            }
        }
    }
    unscale_();
    return status_;
}

} // namespace optimization
} // namespace maths
} // namespace bitsim

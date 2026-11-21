#include "bitsim/control/linear_mpc.h"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace bitsim
{
namespace control
{
namespace
{
constexpr real_t INF = std::numeric_limits<real_t>::infinity();
}

LinearMPC::LinearMPC(uint_t n_states, uint_t n_inputs, uint_t horizon, const QPSettings &settings)
    : n_(n_states), m_(n_inputs), N_(horizon), solver_(settings)
{
    if (n_ == 0 || m_ == 0 || N_ == 0)
    {
        throw std::invalid_argument("LinearMPC: the states, the inputs and the horizon must not "
                                    "be zero");
    }
    u_min_ = Vector::Constant(m_, -INF);
    u_max_ = Vector::Constant(m_, INF);
    x_min_ = Vector::Constant(n_, -INF);
    x_max_ = Vector::Constant(n_, INF);
    x_ref_.assign(N_, Vector::Zero(n_));
    u_ref_.assign(N_, Vector::Zero(m_));
}

void LinearMPC::check_vector_(const Vector &v, uint_t size, const char *what) const
{
    if (static_cast<uint_t>(v.size()) != size)
    {
        throw std::invalid_argument(std::string("LinearMPC: ") + what + " has the wrong size");
    }
}

void LinearMPC::set_model(const Matrix &A, const Matrix &B)
{
    if (static_cast<uint_t>(A.rows()) != n_ || static_cast<uint_t>(A.cols()) != n_ ||
        static_cast<uint_t>(B.rows()) != n_ || static_cast<uint_t>(B.cols()) != m_)
    {
        throw std::invalid_argument("LinearMPC::set_model: A must be n x n and B n x m");
    }
    A_.assign(N_, A);
    B_.assign(N_, B);
    c_.assign(N_, Vector::Zero(n_));
    time_invariant_ = true;
    has_model_ = true;
    structure_dirty_ = true;
    if (has_weights_ && !user_terminal_)
    {
        set_weights(Q_, R_);
    }
}

void LinearMPC::set_model(const std::vector<Matrix> &A, const std::vector<Matrix> &B,
                          const std::vector<Vector> &c)
{
    if (A.size() != N_ || B.size() != N_ || (!c.empty() && c.size() != N_))
    {
        throw std::invalid_argument("LinearMPC::set_model: a matrix A and B, and a vector c, "
                                    "per step of the horizon");
    }
    for (uint_t k = 0; k < N_; ++k)
    {
        if (static_cast<uint_t>(A[k].rows()) != n_ || static_cast<uint_t>(A[k].cols()) != n_ ||
            static_cast<uint_t>(B[k].rows()) != n_ || static_cast<uint_t>(B[k].cols()) != m_ ||
            (!c.empty() && static_cast<uint_t>(c[k].size()) != n_))
        {
            throw std::invalid_argument("LinearMPC::set_model: A must be n x n, B n x m and c "
                                        "of size n");
        }
    }
    A_ = A;
    B_ = B;
    c_ = c.empty() ? std::vector<Vector>(N_, Vector::Zero(n_)) : c;
    time_invariant_ = false;
    has_model_ = true;
    structure_dirty_ = true;
    if (has_weights_ && !user_terminal_)
    {
        P_ = Q_;
    }
}

void LinearMPC::set_weights(const Matrix &Q, const Matrix &R)
{
    if (static_cast<uint_t>(Q.rows()) != n_ || static_cast<uint_t>(Q.cols()) != n_ ||
        static_cast<uint_t>(R.rows()) != m_ || static_cast<uint_t>(R.cols()) != m_)
    {
        throw std::invalid_argument("LinearMPC::set_weights: Q must be n x n and R m x m");
    }
    Q_ = Q;
    R_ = R;
    P_ = Q;
    if (has_model_ && time_invariant_)
    {
        try
        {
            P_ = solve_dare(A_[0], B_[0], Q_, R_);
        }
        catch (const std::runtime_error &)
        {
            // no stabilising Riccati solution: keep Q
            P_ = Q_;
        }
    }
    user_terminal_ = false;
    has_weights_ = true;
    qp_dirty_ = true;
}

void LinearMPC::set_weights(const Matrix &Q, const Matrix &R, const Matrix &P)
{
    set_weights(Q, R);
    if (static_cast<uint_t>(P.rows()) != n_ || static_cast<uint_t>(P.cols()) != n_)
    {
        throw std::invalid_argument("LinearMPC::set_weights: P must be n x n");
    }
    P_ = P;
    user_terminal_ = true;
}

void LinearMPC::set_input_bounds(const Vector &lower, const Vector &upper)
{
    check_vector_(lower, m_, "the lower input bound");
    check_vector_(upper, m_, "the upper input bound");
    if ((lower.array() > upper.array()).any())
    {
        throw std::invalid_argument("LinearMPC::set_input_bounds: lower must not be above upper");
    }
    u_min_ = lower;
    u_max_ = upper;
    qp_dirty_ = true;
}

void LinearMPC::set_state_bounds(const Vector &lower, const Vector &upper, real_t penalty)
{
    if (!(penalty >= 0.0))
    {
        throw std::invalid_argument("LinearMPC::set_state_bounds: the penalty must not be "
                                    "negative");
    }
    state_penalty_ = penalty;
    check_vector_(lower, n_, "the lower state bound");
    check_vector_(upper, n_, "the upper state bound");
    if ((lower.array() > upper.array()).any())
    {
        throw std::invalid_argument("LinearMPC::set_state_bounds: lower must not be above upper");
    }
    x_min_ = lower;
    x_max_ = upper;
    structure_dirty_ = true;
}

void LinearMPC::set_reference(const Vector &x_ref, const Vector &u_ref)
{
    check_vector_(x_ref, n_, "the state reference");
    check_vector_(u_ref, m_, "the input reference");
    x_ref_.assign(N_, x_ref);
    u_ref_.assign(N_, u_ref);
}

void LinearMPC::set_reference(const std::vector<Vector> &x_ref, const std::vector<Vector> &u_ref)
{
    if (x_ref.size() != N_ || u_ref.size() != N_)
    {
        throw std::invalid_argument("LinearMPC::set_reference: a state and an input reference "
                                    "per step of the horizon");
    }
    for (uint_t k = 0; k < N_; ++k)
    {
        check_vector_(x_ref[k], n_, "a state reference");
        check_vector_(u_ref[k], m_, "an input reference");
    }
    x_ref_ = x_ref;
    u_ref_ = u_ref;
}

void LinearMPC::build_structure_()
{
    const auto n = static_cast<Eigen::Index>(n_);
    const auto N = static_cast<Eigen::Index>(N_);
    state_rows_.clear();
    for (Eigen::Index k = 0; k < N; ++k)
    {
        for (Eigen::Index i = 0; i < n; ++i)
        {
            if (std::isfinite(x_min_[i]) || std::isfinite(x_max_[i]))
            {
                state_rows_.push_back(k * n + i);
            }
        }
    }
    structure_dirty_ = false;
    qp_dirty_ = true;
}

void LinearMPC::build_qp_(const Vector &x0)
{
    using Entry = maths::optimization::QPEntry;

    const auto n = static_cast<Eigen::Index>(n_);
    const auto m = static_cast<Eigen::Index>(m_);
    const auto N = static_cast<Eigen::Index>(N_);
    const auto nb = static_cast<Eigen::Index>(state_rows_.size());
    const bool soft = state_penalty_ > 0.0;

    // The variables: the states x_1..x_N, then the inputs u_0..u_{N-1}, then with soft state
    // bounds a slack s >= 0 per bounded state row, which lets it exceed its bounds by s at the
    // cost penalty (s + s^2)
    const Eigen::Index nx = N * n;
    const Eigen::Index nu = N * m;
    const Eigen::Index ns = soft ? nb : 0;
    const Eigen::Index n_vars = nx + nu + ns;

    // The rows: the model, x_{k+1} - A_k x_k - B_k u_k = c_k, with A_0 x_0 moved to the bounds;
    // the inputs; then per bounded state row x in [lo, hi] if hard, or x + s >= lo and
    // x - s <= hi if soft, with s >= 0
    const Eigen::Index r_inputs = nx;
    const Eigen::Index r_states = nx + nu;
    const Eigen::Index n_rows = nx + nu + (soft ? 3 * nb : nb);

    // the cost: sum (x_k - r_k)^T W_k (x_k - r_k) + (u_k - v_k)^T R (u_k - v_k), with W = Q and
    // P at the last step; the QP takes 1/2 z^T Pqp z + q^T z, so Pqp is twice the weights
    Vector q = Vector::Zero(n_vars);
    for (Eigen::Index k = 0; k < N; ++k)
    {
        const Matrix &W = (k == N - 1) ? P_ : Q_;
        q.segment(k * n, n) = -2.0 * W * x_ref_[k];
        q.segment(nx + k * m, m) = -2.0 * R_ * u_ref_[k];
    }
    q.tail(ns).setConstant(state_penalty_);

    Vector l = Vector::Constant(n_rows, -INF);
    Vector u = Vector::Constant(n_rows, INF);
    for (Eigen::Index k = 0; k < N; ++k)
    {
        const Vector rhs = (k == 0) ? Vector(A_[0] * x0 + c_[0]) : c_[k];
        l.segment(k * n, n) = rhs;
        u.segment(k * n, n) = rhs;
        l.segment(r_inputs + k * m, m) = u_min_;
        u.segment(r_inputs + k * m, m) = u_max_;
    }
    for (Eigen::Index j = 0; j < nb; ++j)
    {
        const auto i = state_rows_[j] % n;
        if (soft)
        {
            l[r_states + 2 * j] = x_min_[i];
            u[r_states + 2 * j + 1] = x_max_[i];
            l[r_states + 2 * nb + j] = 0.0;
        }
        else
        {
            l[r_states + j] = x_min_[i];
            u[r_states + j] = x_max_[i];
        }
    }

    if (!qp_dirty_)
    {
        solver_.update(q, l, u);
        return;
    }

    // the quadratic weight of a slack is that of the states, not the penalty: the linear term
    // alone makes the penalty exact, and a large quadratic one leaves the QP so badly scaled that
    // ADMM barely converges
    const real_t slack_weight = std::max(Q_.diagonal().cwiseAbs().maxCoeff(), 1.0e-6);
    std::vector<Entry> p;
    for (Eigen::Index k = 0; k < N; ++k)
    {
        const Matrix &W = (k == N - 1) ? P_ : Q_;
        for (Eigen::Index a = 0; a < n; ++a)
        {
            for (Eigen::Index b = 0; b < n; ++b)
            {
                if (W(a, b) != 0.0)
                {
                    p.push_back({k * n + a, k * n + b, 2.0 * W(a, b)});
                }
            }
        }
        for (Eigen::Index a = 0; a < m; ++a)
        {
            for (Eigen::Index b = 0; b < m; ++b)
            {
                if (R_(a, b) != 0.0)
                {
                    p.push_back({nx + k * m + a, nx + k * m + b, 2.0 * R_(a, b)});
                }
            }
        }
    }
    for (Eigen::Index j = 0; j < ns; ++j)
    {
        p.push_back({nx + nu + j, nx + nu + j, 2.0 * slack_weight});
    }

    std::vector<Entry> c;
    for (Eigen::Index k = 0; k < N; ++k)
    {
        for (Eigen::Index a = 0; a < n; ++a)
        {
            c.push_back({k * n + a, k * n + a, 1.0});
            for (Eigen::Index b = 0; k > 0 && b < n; ++b)
            {
                if (A_[k](a, b) != 0.0)
                {
                    c.push_back({k * n + a, (k - 1) * n + b, -A_[k](a, b)});
                }
            }
            for (Eigen::Index b = 0; b < m; ++b)
            {
                if (B_[k](a, b) != 0.0)
                {
                    c.push_back({k * n + a, nx + k * m + b, -B_[k](a, b)});
                }
            }
        }
        for (Eigen::Index a = 0; a < m; ++a)
        {
            c.push_back({r_inputs + k * m + a, nx + k * m + a, 1.0});
        }
    }
    for (Eigen::Index j = 0; j < nb; ++j)
    {
        const auto col = state_rows_[j];
        if (soft)
        {
            c.push_back({r_states + 2 * j, col, 1.0});
            c.push_back({r_states + 2 * j, nx + nu + j, 1.0});
            c.push_back({r_states + 2 * j + 1, col, 1.0});
            c.push_back({r_states + 2 * j + 1, nx + nu + j, -1.0});
            c.push_back({r_states + 2 * nb + j, nx + nu + j, 1.0});
        }
        else
        {
            c.push_back({r_states + j, col, 1.0});
        }
    }

    solver_.setup(n_vars, p, q, n_rows, c, l, u);
    qp_dirty_ = false;
}

Vector LinearMPC::solve(const Vector &x0)
{
    if (!has_model_ || !has_weights_)
    {
        throw std::logic_error("LinearMPC::solve: set the model and the weights first");
    }
    check_vector_(x0, n_, "x0");

    if (structure_dirty_)
    {
        build_structure_();
    }
    build_qp_(x0);
    solver_.solve();

    const Vector &z = solver_.solution();
    const auto n = static_cast<Eigen::Index>(n_);
    const auto m = static_cast<Eigen::Index>(m_);
    const auto nx = static_cast<Eigen::Index>(N_) * n;
    x_plan_.assign(N_ + 1, Vector());
    u_plan_.assign(N_, Vector());
    x_plan_[0] = x0;
    for (uint_t k = 0; k < N_; ++k)
    {
        const auto kk = static_cast<Eigen::Index>(k);
        x_plan_[k + 1] = z.segment(kk * n, n);
        u_plan_[k] = z.segment(nx + kk * m, m);
    }
    return u_plan_[0];
}

} // namespace control
} // namespace bitsim

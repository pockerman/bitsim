#ifndef EXTENDED_KALMAN_FILTER_H
#define EXTENDED_KALMAN_FILTER_H

#include "Eigen/Dense"
#include "bitsim/bitsim_types.h"

#include <boost/noncopyable.hpp>

#include <concepts>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <tuple>

namespace bitsim
{
namespace estimation
{

///
/// @brief What an update of an ExtendedKalmanFilter did with a measurement
///
struct EKFUpdateResult
{
    bool accepted{false};        ///< false if the gate rejected the measurement
    real_t mahalanobis{0.0};     ///< the squared Mahalanobis distance of the innovation
    DynVec<real_t> innovation{}; ///< z - h(x), or the model's residual of them
};

/**
 * @class ExtendedKalmanFilter
 * @brief The extended Kalman filter: it predicts the state with a non-linear motion model and
 * corrects it with measurements from any number of observation models, each linearised about the
 * current estimate.
 *
 * **The motion model** keeps the state. It has
 * - `evaluate(input)`, which moves the state by one step, and
 * - `get_matrix("F")` and `get_matrix("L")`, the Jacobians of that step with respect to the state
 *   and to the process noise, at the state before the step.
 *
 * predict() then propagates the covariance, \f$P \leftarrow F P F^T + L Q L^T\f$.
 *
 * **An observation model** says what a sensor should measure in a state. It has
 * - `evaluate(x)`, the predicted measurement \f$h(x)\f$ for the state as a vector, and
 * - `get_matrix("H")`, the Jacobian of h with respect to the state, valid after `evaluate`;
 * - optionally `has_matrix(name)`: if it says it has "M", `get_matrix("M")` is the Jacobian with
 *   respect to the measurement noise; otherwise that is the identity;
 * - optionally `residual(z, zpred)`, for measurements such as angles whose difference needs
 *   wrapping.
 *
 * update(model, z, R, gate) corrects the state with a measurement z of noise covariance R:
 *
 * \f[
 * S = H P H^T + M R M^T, \quad K = P H^T S^{-1}, \quad x \leftarrow x + K (z - h(x)), \quad
 * P \leftarrow (I - K H) P
 * \f]
 *
 * unless the squared Mahalanobis distance of the innovation, \f$\nu^T S^{-1} \nu\f$, is above the
 * gate: then it leaves the state alone, which keeps an outlier, a sonar echo off something the map
 * does not have, from pulling the estimate away. Each sensor can have its own model, noise and
 * gate, and be used whenever it has a reading.
 *
 * The filter can also hold one observation model with its R, as set_matrix("R"), for update(z).
 */
template <typename MotionModelTp, typename ObservationModelTp = void>
class ExtendedKalmanFilter : private boost::noncopyable
{
  public:
    typedef MotionModelTp motion_model_type;
    typedef ObservationModelTp observation_model_type;
    typedef typename motion_model_type::input_type motion_model_input_type;
    typedef typename motion_model_type::matrix_type matrix_type;
    typedef typename motion_model_type::state_type state_type;

    ///
    /// @brief A filter for the motion model, with the observation models given to update()
    ///
    explicit ExtendedKalmanFilter(motion_model_type &motion_model);

    ///
    /// @brief A filter with an observation model of its own, for update(z)
    ///
    template <typename O = ObservationModelTp>
    ExtendedKalmanFilter(motion_model_type &motion_model, const O &observation_model);

    ///
    /// @brief Predict the state one step with the input, and its covariance with the Q set by
    /// set_matrix("Q")
    ///
    void predict(const motion_model_input_type &input);

    ///
    /// @brief Predict with the process noise covariance Q of this step, for noise that depends on
    /// the input
    ///
    void predict(const motion_model_input_type &input, const matrix_type &Q);

    ///
    /// @brief Correct the state with the measurement z of the observation model, of noise
    /// covariance R. A measurement whose squared Mahalanobis distance is above gate is not used
    ///
    template <typename ObservationModel>
    EKFUpdateResult update(const ObservationModel &model, const DynVec<real_t> &z,
                           const matrix_type &R,
                           real_t gate = std::numeric_limits<real_t>::infinity());

    ///
    /// @brief Correct the state with the measurement z of the filter's own observation model and
    /// the R set by set_matrix("R")
    ///
    EKFUpdateResult update(const DynVec<real_t> &z);

    ///
    /// @brief predict(), then update(z)
    ///
    void estimate(const std::tuple<motion_model_input_type, DynVec<real_t>> &input);

    ///
    /// @brief Set P, the covariance of the state, Q, of the process noise, or R, of the own
    /// observation model's noise
    ///
    void set_matrix(const std::string &name, const matrix_type &mat);
    ExtendedKalmanFilter &with_matrix(const std::string &name, const matrix_type &mat);
    bool has_matrix(const std::string &name) const;
    const matrix_type &operator[](const std::string &name) const;
    matrix_type &operator[](const std::string &name);

    const state_type &get_state() const { return motion_model_ptr_->get_state(); }
    state_type &get_state() { return motion_model_ptr_->get_state(); }

    ///
    /// @brief The state as a vector
    ///
    DynVec<real_t> state_vector() const { return motion_model_ptr_->get_state().as_vector(); }

    real_t get(const std::string &name) const
    {
        return motion_model_ptr_->get_state_property(name);
    }

  protected:
    motion_model_type *motion_model_ptr_;
    const void *observation_model_ptr_{nullptr};
    std::map<std::string, matrix_type> matrices_;
};

namespace ekf_detail
{
template <typename Model>
concept CanSayIfItHasMatrices = requires(const Model &m)
{
    {
        m.has_matrix("M")
        } -> std::convertible_to<bool>;
};

template <typename Model>
concept HasResidual = requires(const Model &m, const DynVec<real_t> &z)
{
    m.residual(z, z);
};
} // namespace ekf_detail

template <typename MotionModelTp, typename ObservationModelTp>
ExtendedKalmanFilter<MotionModelTp, ObservationModelTp>::ExtendedKalmanFilter(
    motion_model_type &motion_model)
    : motion_model_ptr_(&motion_model)
{
}

template <typename MotionModelTp, typename ObservationModelTp>
template <typename O>
ExtendedKalmanFilter<MotionModelTp, ObservationModelTp>::ExtendedKalmanFilter(
    motion_model_type &motion_model, const O &observation_model)
    : motion_model_ptr_(&motion_model), observation_model_ptr_(&observation_model)
{
}

template <typename MotionModelTp, typename ObservationModelTp>
void ExtendedKalmanFilter<MotionModelTp, ObservationModelTp>::set_matrix(const std::string &name,
                                                                         const matrix_type &mat)
{
    if (name != "Q" && name != "K" && name != "R" && name != "P")
    {
        throw std::logic_error("Invalid matrix name. Name: " + name + " not in [Q, K, R, P]");
    }
    matrices_.insert_or_assign(name, mat);
}

template <typename MotionModelTp, typename ObservationModelTp>
ExtendedKalmanFilter<MotionModelTp, ObservationModelTp> &
ExtendedKalmanFilter<MotionModelTp, ObservationModelTp>::with_matrix(const std::string &name,
                                                                     const matrix_type &mat)
{
    set_matrix(name, mat);
    return *this;
}

template <typename MotionModelTp, typename ObservationModelTp>
bool ExtendedKalmanFilter<MotionModelTp, ObservationModelTp>::has_matrix(
    const std::string &name) const
{
    return matrices_.find(name) != matrices_.end();
}

template <typename MotionModelTp, typename ObservationModelTp>
const typename ExtendedKalmanFilter<MotionModelTp, ObservationModelTp>::matrix_type &
ExtendedKalmanFilter<MotionModelTp, ObservationModelTp>::operator[](const std::string &name) const
{
    auto itr = matrices_.find(name);
    if (itr == matrices_.end())
    {
        throw std::invalid_argument("Matrix: " + name + " does not exist");
    }
    return itr->second;
}

template <typename MotionModelTp, typename ObservationModelTp>
typename ExtendedKalmanFilter<MotionModelTp, ObservationModelTp>::matrix_type &
ExtendedKalmanFilter<MotionModelTp, ObservationModelTp>::operator[](const std::string &name)
{
    auto itr = matrices_.find(name);
    if (itr == matrices_.end())
    {
        throw std::invalid_argument("Matrix: " + name + " does not exist");
    }
    return itr->second;
}

template <typename MotionModelTp, typename ObservationModelTp>
void ExtendedKalmanFilter<MotionModelTp, ObservationModelTp>::predict(
    const motion_model_input_type &u)
{
    predict(u, (*this)["Q"]);
}

template <typename MotionModelTp, typename ObservationModelTp>
void ExtendedKalmanFilter<MotionModelTp, ObservationModelTp>::predict(
    const motion_model_input_type &u, const matrix_type &Q)
{
    // The motion model moves the state and works out F and L at the state before the move
    motion_model_ptr_->evaluate(u);
    auto &P = (*this)["P"];
    const auto &L = motion_model_ptr_->get_matrix("L");
    const auto &F = motion_model_ptr_->get_matrix("F");
    P = F * P * F.transpose() + L * Q * L.transpose();
}

template <typename MotionModelTp, typename ObservationModelTp>
template <typename ObservationModel>
EKFUpdateResult ExtendedKalmanFilter<MotionModelTp, ObservationModelTp>::update(
    const ObservationModel &model, const DynVec<real_t> &z, const matrix_type &R, real_t gate)
{
    auto &state = motion_model_ptr_->get_state();
    auto &P = (*this)["P"];

    const DynVec<real_t> zpred = model.evaluate(state.as_vector());
    const matrix_type H = model.get_matrix("H");
    if (z.size() != zpred.size() || H.rows() != z.size() || H.cols() != P.rows())
    {
        throw std::runtime_error("ExtendedKalmanFilter::update: the measurement has " +
                                 std::to_string(z.size()) + " values, the model predicts " +
                                 std::to_string(zpred.size()) + " and H is " +
                                 std::to_string(H.rows()) + " x " + std::to_string(H.cols()));
    }

    matrix_type MRM = R;
    if constexpr (ekf_detail::CanSayIfItHasMatrices<ObservationModel>)
    {
        if (model.has_matrix("M"))
        {
            const matrix_type M = model.get_matrix("M");
            MRM = M * R * M.transpose();
        }
    }

    EKFUpdateResult result;
    if constexpr (ekf_detail::HasResidual<ObservationModel>)
    {
        result.innovation = model.residual(z, zpred);
    }
    else
    {
        result.innovation = z - zpred;
    }

    const matrix_type S = H * P * H.transpose() + MRM;
    const auto S_ldlt = S.ldlt();
    const ColVec<real_t> nu = result.innovation.transpose();
    result.mahalanobis = nu.dot(S_ldlt.solve(nu));
    if (!(result.mahalanobis <= gate))
    {
        return result;
    }

    // K = P H^T S^-1, worked out as (S^-1 H P)^T since S and P are symmetric
    const matrix_type K = S_ldlt.solve(H * P).transpose();
    const ColVec<real_t> dx = K * nu;
    state += dx.transpose();

    const auto n = P.rows();
    P = (matrix_type::Identity(n, n) - K * H) * P;
    // Keep P symmetric against round-off
    P = 0.5 * (P + P.transpose()).eval();

    if (has_matrix("K"))
    {
        (*this)["K"] = K;
    }
    else
    {
        set_matrix("K", K);
    }
    result.accepted = true;
    return result;
}

template <typename MotionModelTp, typename ObservationModelTp>
EKFUpdateResult
ExtendedKalmanFilter<MotionModelTp, ObservationModelTp>::update(const DynVec<real_t> &z)
{
    if constexpr (std::is_void_v<ObservationModelTp>)
    {
        throw std::logic_error("ExtendedKalmanFilter::update(z) needs a filter made with an "
                               "observation model; use update(model, z, R)");
    }
    else
    {
        if (observation_model_ptr_ == nullptr)
        {
            throw std::logic_error("ExtendedKalmanFilter::update(z): no observation model");
        }
        return update(*static_cast<const ObservationModelTp *>(observation_model_ptr_), z,
                      (*this)["R"]);
    }
}

template <typename MotionModelTp, typename ObservationModelTp>
void ExtendedKalmanFilter<MotionModelTp, ObservationModelTp>::estimate(
    const std::tuple<motion_model_input_type, DynVec<real_t>> &input)
{
    predict(std::get<0>(input));
    update(std::get<1>(input));
}

} // namespace estimation
} // namespace bitsim

#endif

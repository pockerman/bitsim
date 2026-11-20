#ifndef BITSIM_KINEMATICS_SPATIAL_ARM_H
#define BITSIM_KINEMATICS_SPATIAL_ARM_H

#include "bitsim/bitsim_config.h"

#ifdef BITSIM_CHRONO

#include "bitsim/bitsim_types.h"

#include <Eigen/Core>
#include <chrono/core/ChFrame.h>
#include <chrono/core/ChVector3.h>

#include <array>
#include <functional>
#include <optional>
#include <vector>

namespace bitsim
{
namespace kinematics
{

///
/// @ingroup bitsim_kinematics
/// @brief An arm with three revolute joints. Joint 1 turns the arm about the vertical axis at
/// height h0; joints 2 and 3, the shoulder and the elbow, turn about horizontal axes, so the
/// upper arm and the forearm move in a vertical plane
///
struct SpatialArm
{
    real_t h0{0.3}; ///< the height of the shoulder above the floor [m]
    real_t l1{0.4}; ///< the upper arm [m]
    real_t l2{0.3}; ///< the forearm [m]
};

///
/// @ingroup bitsim_kinematics
/// @brief The angles of the three joints of a SpatialArm [rad]
///
using SpatialJoints = std::array<real_t, 3>;

///
/// @ingroup bitsim_kinematics
/// @brief The pose in the world of every joint frame, followed by the end-effector:
/// \f$X_{W1}, X_{W2}, X_{W3}, X_{WE}\f$
///
std::vector<chrono::ChFrame<>> spatial_fk_all(const SpatialArm &arm, const SpatialJoints &q);

///
/// @ingroup bitsim_kinematics
/// @brief The pose of the end-effector in the world, \f$X_{WE}\f$
///
chrono::ChFrame<> spatial_fk(const SpatialArm &arm, const SpatialJoints &q);

///
/// @ingroup bitsim_kinematics
/// @brief All the joint angles that put the end-effector at p_W.
///
/// Joint 1 points the vertical plane of the arm at the target, and in that plane the shoulder
/// and the elbow are a PlanarArm. The arm can also turn its back to the target and reach over
/// its head, so there are up to four solutions; none when p_W is out of reach
///
std::vector<SpatialJoints> spatial_ik(const SpatialArm &arm, const chrono::ChVector3d &p_W);

///
/// @ingroup bitsim_kinematics
/// @brief The solution facing the target with the elbow up, or nothing when p_W is out of reach
///
std::optional<SpatialJoints> spatial_ik_elbow_up(const SpatialArm &arm,
                                                 const chrono::ChVector3d &p_W);

///
/// @ingroup bitsim_kinematics
/// @brief The Jacobian \f$\partial(x, y, z) / \partial(q_1, q_2, q_3)\f$ of the end-effector's
/// position by central differences. It needs only the forward kinematics
///
Eigen::Matrix3d numerical_jacobian(const SpatialArm &arm, const SpatialJoints &q);

///
/// @ingroup bitsim_kinematics
/// @brief The result of numerical_ik
///
struct IKResult
{
    SpatialJoints q{};     ///< the joint angles reached [rad]
    int iterations{0};     ///< the iterations it took
    real_t error{0.0};     ///< the distance from the end-effector to the target [m]
    bool converged{false}; ///< whether the error is below the tolerance
};

///
/// @ingroup bitsim_kinematics
/// @brief Called by numerical_ik at every iteration with the iteration, the joint angles and
/// the error [m], for example to print the progress
///
using IKObserver = std::function<void(int, const SpatialJoints &, real_t)>;

///
/// @ingroup bitsim_kinematics
/// @brief Inverse kinematics by iteration from q0: \f$q \leftarrow q + \Delta q\f$ until the
/// end-effector is within tolerance of the target.
///
/// With damping \f$\lambda = 0\f$ the step is Newton's, \f$\Delta q = J^{-1} e\f$; otherwise
/// it is damped least squares, \f$\Delta q = J^T (J J^T + \lambda^2 I)^{-1} e\f$, which stays
/// finite near a singular J. No joint turns by more than max_step [rad] in one step. The
/// iteration stops too when the arm stops moving, which is the best it can do for a target out
/// of reach
///
IKResult numerical_ik(const SpatialArm &arm, const chrono::ChVector3d &target, SpatialJoints q0,
                      real_t damping, int max_iterations = 100, real_t tolerance = 1.0e-6,
                      real_t max_step = 0.5, const IKObserver &observer = {});

} // namespace kinematics
} // namespace bitsim

#endif // BITSIM_CHRONO
#endif // BITSIM_KINEMATICS_SPATIAL_ARM_H

#ifndef BITSIM_KINEMATICS_PLANAR_ARM_H
#define BITSIM_KINEMATICS_PLANAR_ARM_H

#include "bitsim/bitsim_config.h"

#ifdef BITSIM_CHRONO

#include "bitsim/bitsim_types.h"

#include <Eigen/Core>
#include <chrono/core/ChFrame.h>
#include <chrono/core/ChVector3.h>

#include <array>
#include <vector>

namespace bitsim
{
namespace kinematics
{

///
/// @ingroup bitsim_kinematics
/// @brief A planar arm with two links that moves in the xy plane, like a SCARA arm seen from
/// above. Joint 1 is at the origin of the world, and both joints turn about z
///
struct PlanarArm
{
    real_t l1{0.4}; ///< the first link [m]
    real_t l2{0.3}; ///< the second link [m]
};

///
/// @ingroup bitsim_kinematics
/// @brief The angles of the two joints of a PlanarArm [rad]
///
using PlanarJoints = std::array<real_t, 2>;

///
/// @ingroup bitsim_kinematics
/// @brief Where the tip is, written out with trigonometry:
/// \f$x = l_1 \cos q_1 + l_2 \cos(q_1 + q_2)\f$, \f$y = l_1 \sin q_1 + l_2 \sin(q_1 + q_2)\f$
///
chrono::ChVector3d planar_fk(const PlanarArm &arm, const PlanarJoints &q);

///
/// @ingroup bitsim_kinematics
/// @brief The pose of the tip as a chain of frames, \f$X_{WE} = X_{W1} X_{12} X_{2E}\f$:
/// each joint adds a rotation about z, each link an offset along x
///
chrono::ChFrame<> planar_fk_frames(const PlanarArm &arm, const PlanarJoints &q);

///
/// @ingroup bitsim_kinematics
/// @brief The joint angles that put the tip at (x, y), from the law of cosines.
///
/// @return no solution when (x, y) is out of reach, otherwise the two solutions with the elbow
/// on either side, elbow angle positive first. When the arm is fully stretched or folded the
/// two are the same and only one is returned. The angles are in \f$(-\pi, \pi]\f$
///
std::vector<PlanarJoints> planar_ik(const PlanarArm &arm, real_t x, real_t y);

///
/// @ingroup bitsim_kinematics
/// @brief The Jacobian \f$\partial(x, y) / \partial(q_1, q_2)\f$. It is singular when
/// \f$\sin q_2 = 0\f$: the arm fully stretched or folded
///
Eigen::Matrix2d planar_jacobian(const PlanarArm &arm, const PlanarJoints &q);

} // namespace kinematics
} // namespace bitsim

#endif // BITSIM_CHRONO
#endif // BITSIM_KINEMATICS_PLANAR_ARM_H

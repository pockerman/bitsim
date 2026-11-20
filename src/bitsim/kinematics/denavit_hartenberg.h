#ifndef BITSIM_KINEMATICS_DENAVIT_HARTENBERG_H
#define BITSIM_KINEMATICS_DENAVIT_HARTENBERG_H

#include "bitsim/bitsim_config.h"

#ifdef BITSIM_CHRONO

#include "bitsim/bitsim_types.h"
#include "bitsim/kinematics/spatial_arm.h"

#include <chrono/core/ChFrame.h>

#include <vector>

namespace bitsim
{
namespace kinematics
{

///
/// @ingroup bitsim_kinematics
/// @brief One row of a Denavit-Hartenberg table: the pose of a joint frame in the one before
/// it is \f$R_z(\theta)\, T_z(d)\, T_x(a)\, R_x(\alpha)\f$
///
struct DHRow
{
    real_t theta{0.0}; ///< rotation about z [rad]; the joint angle of a revolute joint
    real_t d{0.0};     ///< offset along z [m]
    real_t a{0.0};     ///< offset along the new x [m]
    real_t alpha{0.0}; ///< rotation about the new x [rad]
};

///
/// @ingroup bitsim_kinematics
/// @brief \f$R_z(\theta)\, T_z(d)\, T_x(a)\, R_x(\alpha)\f$
///
chrono::ChFrame<> dh_frame(const DHRow &row);

///
/// @ingroup bitsim_kinematics
/// @brief The product of the frames of all the rows: the pose of the last frame in the first
///
chrono::ChFrame<> dh_fk(const std::vector<DHRow> &table);

///
/// @ingroup bitsim_kinematics
/// @brief The Denavit-Hartenberg table of a SpatialArm at joint angles q. dh_fk of it is
/// spatial_fk
///
std::vector<DHRow> spatial_dh_table(const SpatialArm &arm, const SpatialJoints &q);

} // namespace kinematics
} // namespace bitsim

#endif // BITSIM_CHRONO
#endif // BITSIM_KINEMATICS_DENAVIT_HARTENBERG_H

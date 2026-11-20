#ifndef BITSIM_KINEMATICS_JOINT_H
#define BITSIM_KINEMATICS_JOINT_H

#include "bitsim/bitsim_config.h"

#ifdef BITSIM_CHRONO

#include "bitsim/bitsim_types.h"

#include <chrono/core/ChFrame.h>

namespace bitsim
{
namespace kinematics
{

///
/// @ingroup bitsim_kinematics
/// @brief The frame of a revolute joint turned by q [rad] about its z axis: a rotation, with
/// no offset
///
chrono::ChFrame<> joint_rotation(real_t q);

} // namespace kinematics
} // namespace bitsim

#endif // BITSIM_CHRONO
#endif // BITSIM_KINEMATICS_JOINT_H

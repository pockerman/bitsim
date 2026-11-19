#ifndef BITRL_KINEMATICS_JOINT_H
#define BITRL_KINEMATICS_JOINT_H

#include "bitrl/bitrl_config.h"

#ifdef BITRL_CHRONO

#include "bitrl/bitrl_types.h"

#include <chrono/core/ChFrame.h>

namespace bitrl
{
namespace kinematics
{

///
/// @ingroup bitrl_kinematics
/// @brief The frame of a revolute joint turned by q [rad] about its z axis: a rotation, with
/// no offset
///
chrono::ChFrame<> joint_rotation(real_t q);

} // namespace kinematics
} // namespace bitrl

#endif // BITRL_CHRONO
#endif // BITRL_KINEMATICS_JOINT_H

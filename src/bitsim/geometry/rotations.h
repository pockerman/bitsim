#ifndef BITSIM_GEOMETRY_ROTATIONS_H
#define BITSIM_GEOMETRY_ROTATIONS_H

/**
 * \file
 * \brief Rotation matrices: the elementary rotations, checks, and yaw, pitch
 * and roll angles. Needs bitsim built with Chrono.
 *
 * The convention is that of the frames examples: \f$R_{AB}\f$ is the
 * orientation of frame B expressed in frame A, so that
 * \f$R_{WR} R_{RS} = R_{WS}\f$ and \f$R_{WR}\, v_R = v_W\f$.
 */

#include "bitsim/bitsim_config.h"

#ifdef BITSIM_CHRONO

#include "bitsim/bitsim_types.h"

#include <chrono/core/ChMatrix33.h>

namespace bitsim
{
namespace geometry
{

///
/// @brief The rotation by angle [rad] about the x axis. It leaves the x axis unchanged:
///
/// \f[
/// R_x(\theta) = \begin{bmatrix} 1 & 0 & 0 \\ 0 & \cos\theta & -\sin\theta \\ 0 & \sin\theta &
/// \cos\theta \end{bmatrix} \f]
///
chrono::ChMatrix33d rot_x(real_t angle);

///
/// @brief The rotation by angle [rad] about the y axis
///
/// \f[
/// R_y(\theta) = \begin{bmatrix} \cos\theta & 0 & \sin\theta \\ 0 & 1 & 0 \\ -\sin\theta & 0 &
/// \cos\theta \end{bmatrix} \f]
///
chrono::ChMatrix33d rot_y(real_t angle);

///
/// @brief The rotation by angle [rad] about the z axis
///
/// \f[
/// R_z(\theta) = \begin{bmatrix} \cos\theta & -\sin\theta & 0 \\ \sin\theta & \cos\theta & 0 \\ 0 &
/// 0 & 1 \end{bmatrix} \f]
///
chrono::ChMatrix33d rot_z(real_t angle);

///
/// @brief Whether R is a rotation: \f$R^T R = I\f$ and \f$\det R = +1\f$, to within
/// tolerance. An orthogonal matrix with determinant -1 is a mirror image, which no body can do
///
bool is_rotation(const chrono::ChMatrix33d &R, real_t tolerance = 1.0e-9);

///
/// @brief Whether A and B are equal to within tolerance, in the Frobenius norm
///
bool same_matrix(const chrono::ChMatrix33d &A, const chrono::ChMatrix33d &B,
                 real_t tolerance = 1.0e-9);

///
/// @brief Yaw, pitch and roll angles [rad]: the rotation is
/// \f$R_z(\mathrm{yaw})\, R_y(\mathrm{pitch})\, R_x(\mathrm{roll})\f$
///
struct YawPitchRoll
{
    real_t yaw{0.0};
    real_t pitch{0.0};
    real_t roll{0.0};
};

///
/// @brief The rotation \f$R_z(\mathrm{yaw})\, R_y(\mathrm{pitch})\, R_x(\mathrm{roll})\f$
///
chrono::ChMatrix33d from_yaw_pitch_roll(const YawPitchRoll &angles);

///
/// @brief The inverse of from_yaw_pitch_roll, with the pitch in [-90, 90] degrees. It is not
/// defined at a pitch of +-90 degrees, where yaw and roll turn about the same axis (gimbal lock)
///
YawPitchRoll to_yaw_pitch_roll(const chrono::ChMatrix33d &R);

} // namespace geometry
} // namespace bitsim

#endif // BITSIM_CHRONO
#endif // BITSIM_GEOMETRY_ROTATIONS_H

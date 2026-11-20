#ifndef BITSIM_KINEMATICS_MODULE_H
#define BITSIM_KINEMATICS_MODULE_H

/**
 * @defgroup bitsim_kinematics bitsim::kinematics
 * @brief Forward and inverse kinematics of serial arms. Needs bitsim built with Chrono.
 *
 * Poses follow the convention of the frames examples: \f$X_{AB}\f$ is the pose of frame B
 * expressed in frame A, so that \f$X_{WR} X_{RS} = X_{WS}\f$. Every joint is revolute and
 * turns about its own z axis, as Chrono's rotational motors do.
 *
 * - PlanarArm: two links in the xy plane, like a SCARA arm seen from above;
 * - SpatialArm: a base that turns about the vertical, a shoulder and an elbow;
 * - DHRow: Denavit-Hartenberg parameters, for any serial chain of revolute joints.
 */

#endif // BITSIM_KINEMATICS_MODULE_H

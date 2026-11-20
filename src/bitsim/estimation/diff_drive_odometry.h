#ifndef DIFF_DRIVE_ODOMETRY_H
#define DIFF_DRIVE_ODOMETRY_H

#include "bitsim/bitsim_types.h"

namespace bitsim
{
namespace estimation
{

/**
 * @class DiffDriveOdometry
 * @brief Dead reckoning for a differential drive robot from its wheel
 * encoders.
 *
 * Between two updates the left and right wheels turn by \f$\Delta\theta_L\f$
 * and \f$\Delta\theta_R\f$, so with wheel radius \f$r\f$ and the wheels
 * \f$L\f$ apart the robot drives and turns by
 *
 * \f[
 * \Delta s = \frac{r (\Delta\theta_L + \Delta\theta_R)}{2}, \qquad
 * \Delta\psi = \frac{r (\Delta\theta_R - \Delta\theta_L)}{L}
 * \f]
 *
 * and its pose moves along the heading halfway through the step:
 *
 * \f[
 * x \mathrel{+}= \Delta s \cos\bar\psi, \quad y \mathrel{+}= \Delta s \sin\bar\psi, \quad
 * \bar\psi = \psi + \Delta\psi / 2
 * \f]
 *
 * The heading can come from the wheels too, update(left, right), or from
 * another sensor such as a gyroscope, update(left, right, heading). Wheels that
 * slip, or a radius or axle length that is a little off, turn into errors of
 * the heading that grow without bound; a gyroscope does better for heading.
 */
class DiffDriveOdometry
{
  public:
    ///
    /// @brief A pose in the plane: the point halfway between the wheels and
    /// the heading from the x axis [m, m, rad]
    ///
    struct Pose
    {
        real_t x{0.0};
        real_t y{0.0};
        real_t yaw{0.0};
    };

    ///
    /// @param wheel_radius The radius of the wheels [m]
    /// @param axle_length The distance between the wheels [m]
    ///
    DiffDriveOdometry(real_t wheel_radius, real_t axle_length);

    ///
    /// @brief Start from a pose, with the wheels at the given angles [rad]
    ///
    void reset(const Pose &pose, real_t left_angle, real_t right_angle);

    ///
    /// @brief Move the pose by what the wheels turned since the last update,
    /// heading included
    /// @param left_angle The angle of the left wheel now [rad]
    /// @param right_angle The angle of the right wheel now [rad]
    ///
    const Pose &update(real_t left_angle, real_t right_angle);

    ///
    /// @brief Move the pose by the distance the wheels drove, along a heading
    /// measured otherwise
    /// @param left_angle The angle of the left wheel now [rad]
    /// @param right_angle The angle of the right wheel now [rad]
    /// @param heading The heading now, for example from a gyroscope [rad]
    ///
    const Pose &update(real_t left_angle, real_t right_angle, real_t heading);

    const Pose &pose() const noexcept { return pose_; }

    ///
    /// @return The length of the path driven since reset() [m]. Turning on the
    /// spot adds nothing
    ///
    real_t distance() const noexcept { return distance_; }

    real_t wheel_radius() const noexcept { return wheel_radius_; }
    real_t axle_length() const noexcept { return axle_length_; }

  private:
    real_t step_(real_t left_angle, real_t right_angle, real_t &turned);

    real_t wheel_radius_;
    real_t axle_length_;
    Pose pose_;
    real_t left_{0.0};
    real_t right_{0.0};
    real_t distance_{0.0};
};

} // namespace estimation
} // namespace bitsim

#endif // DIFF_DRIVE_ODOMETRY_H

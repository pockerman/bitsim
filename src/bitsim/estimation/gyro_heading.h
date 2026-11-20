#ifndef BITSIM_ESTIMATION_GYRO_HEADING_H
#define BITSIM_ESTIMATION_GYRO_HEADING_H

#include "bitsim/bitsim_types.h"

namespace bitsim
{
namespace estimation
{

/**
 * @class GyroHeading
 * @brief The heading of a robot from its gyroscope: dead reckoning of the rate about the
 * vertical.
 *
 * Adding up rate x time from a known start gives the heading,
 *
 * \f[
 * \psi_{k+1} = \psi_k + (\omega_{z,k} - b)\, \Delta t
 * \f]
 *
 * and an error in the rate never goes away: a bias b that is not taken off makes the heading
 * drift by b t. To keep the drift small the robot can first stand still for a calibration
 * time. It is not turning then, so the mean of what the gyroscope reads is its bias, which is
 * taken off every reading after that.
 */
class GyroHeading
{
  public:
    ///
    /// @param start_heading The heading the robot starts at [rad]
    /// @param calibration_time How long the robot stands still to measure the bias [s]; 0 for
    /// no calibration, with a bias of 0
    ///
    explicit GyroHeading(real_t start_heading, real_t calibration_time = 1.0);

    ///
    /// @brief True while the robot should stand still for the bias to be measured
    ///
    bool is_calibrating() const noexcept { return calibrated_for_ < calibration_time_; }

    ///
    /// @brief Take a reading of the gyroscope's vertical axis [rad/s], dt [s] after the last
    ///
    void update(real_t gyro_z, real_t dt);

    ///
    /// @brief The heading [rad], in \f$[-\pi, \pi]\f$
    ///
    real_t heading() const noexcept { return heading_; }

    ///
    /// @brief The bias measured while calibrating [rad/s]
    ///
    real_t bias() const noexcept { return bias_; }

  private:
    real_t heading_;
    real_t calibration_time_;
    real_t calibrated_for_{0.0};
    real_t rate_sum_{0.0};
    real_t bias_{0.0};
};

} // namespace estimation
} // namespace bitsim

#endif // BITSIM_ESTIMATION_GYRO_HEADING_H

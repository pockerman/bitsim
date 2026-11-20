#include "bitsim/estimation/gyro_heading.h"
#include "bitsim/bitsim_consts.h"

#include <cmath>
#include <stdexcept>

namespace bitsim
{
namespace estimation
{

GyroHeading::GyroHeading(real_t start_heading, real_t calibration_time)
    : heading_(start_heading), calibration_time_(calibration_time)
{
    if (calibration_time < 0.0)
    {
        throw std::invalid_argument("GyroHeading: the calibration time cannot be negative");
    }
}

void GyroHeading::update(real_t gyro_z, real_t dt)
{
    if (is_calibrating())
    {
        rate_sum_ += gyro_z * dt;
        calibrated_for_ += dt;
        if (!is_calibrating())
        {
            bias_ = rate_sum_ / calibrated_for_;
        }
        return;
    }
    heading_ = std::remainder(heading_ + (gyro_z - bias_) * dt, 2.0 * consts::maths::PI);
}

} // namespace estimation
} // namespace bitsim

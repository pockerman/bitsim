#ifndef IMU_H
#define IMU_H

#include "bitsim/bitsim_types.h"
#include "bitsim/sensors/messages/message_base.h"

#include <array>
#include <optional>
#include <string>

namespace bitsim
{
namespace sensors
{

/**
 * @struct IMUMessage
 * @brief Models a message from an inertial measurement unit with a three axis
 * accelerometer and a three axis gyroscope:
 *
 * @code
 * {"accel": [0.12, -0.03, 9.79], "gyro": [0.001, -0.002, 0.35]}
 * @endcode
 *
 * `accel` is the specific force [m/s^2]: what the accelerometer measures, the
 * acceleration minus gravity, so 9.81 up at rest. `gyro` is the angular
 * velocity [rad/s]. Both are along the axes of the IMU. `timestamp`, in UTC,
 * is optional.
 */
struct IMUMessage : MessageBase
{
    std::array<real_t, 3> accel{0.0, 0.0, 0.0};
    std::array<real_t, 3> gyro{0.0, 0.0, 0.0};

    /// @return The message, or nothing if the text is not a valid message
    static std::optional<IMUMessage> parse(const std::string &msg);
};

} // namespace sensors
} // namespace bitsim

#endif // IMU_H

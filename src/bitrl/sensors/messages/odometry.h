#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "bitrl/bitrl_types.h"
#include "bitrl/sensors/messages/message_base.h"

#include <optional>
#include <string>

namespace bitrl
{
namespace sensors
{

/**
 * @struct OdometryMessage
 * @brief A pose estimate from a board:
 *
 * @code
 * {"x": 1.25, "y": -0.40, "yaw": 0.52, "timestamp": "2026-11-18 10:15:02"}
 * @endcode
 *
 * `x` and `y` in metres and `yaw` in radians are needed. `timestamp`, in UTC, is optional; without
 * it the message counts from when it arrived.
 */
struct OdometryMessage : MessageBase
{
    real_t x{0.0};   ///< [m]
    real_t y{0.0};   ///< [m]
    real_t yaw{0.0}; ///< [rad]

    /// @return The message, or nothing if the text is not a valid message
    static std::optional<OdometryMessage> parse(const std::string &msg);
};
} // namespace sensors
} // namespace bitrl

#endif // ODOMETRY_H

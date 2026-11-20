//
// Created by alex on 11/23/25.
//

#ifndef ULTRASOUND_H
#define ULTRASOUND_H

#include "bitsim/bitsim_types.h"
#include "bitsim/sensors/messages/message_base.h"
#include <optional>
#include <string>

namespace bitsim
{
namespace sensors
{

/**
 * @struct UltrasoundMessage
 * @brief Models a message from an ultrasound sensor:
 *
 * @code
 * {"distance": 62.0, "unit": "cm", "timestamp": "2026-11-18 10:15:02"}
 * @endcode
 *
 * `unit` is m, cm or mm, m if it is missing. `timestamp`, in UTC, is optional.
 */
struct UltrasoundMessage : MessageBase
{
    /// The distance as the message gives it, in unit_str
    real_t distance{0.0};

    /// The unit of distance
    std::string unit_str{"m"};

    /// @return The distance in metres
    real_t metres() const;

    /// @return The message, or nothing if the text is not a valid message
    static std::optional<UltrasoundMessage> parse(const std::string &msg);
};
} // namespace sensors
} // namespace bitsim

#endif // ULTRASOUND_H

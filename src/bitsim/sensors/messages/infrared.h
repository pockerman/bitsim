#ifndef INFRARED_H
#define INFRARED_H

#include "bitsim/bitsim_types.h"
#include "bitsim/sensors/messages/message_base.h"

#include <optional>
#include <string>
#include <vector>

namespace bitsim
{
namespace sensors
{

/**
 * @struct InfraredMessage
 * @brief Models a message from an array of infrared reflectance sensors, one
 * value per element:
 *
 * @code
 * {"values": [0.91, 0.12, 0.88], "timestamp": "2026-11-18 10:15:02"}
 * @endcode
 *
 * Each value is the share of the infrared light that the floor under the
 * element reflects back, from 0 for black to 1 for white. A board that reads
 * the elements with an ADC divides by the reading over white paper.
 * `timestamp`, in UTC, is optional.
 */
struct InfraredMessage : MessageBase
{
    /// The reflectance under each element, in [0, 1]
    std::vector<real_t> values;

    /// @return The message, or nothing if the text is not a valid message
    static std::optional<InfraredMessage> parse(const std::string &msg);
};

} // namespace sensors
} // namespace bitsim

#endif // INFRARED_H

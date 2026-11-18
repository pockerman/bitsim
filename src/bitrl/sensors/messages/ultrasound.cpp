#include "bitrl/sensors/messages/ultrasound.h"
#include "bitrl/bitrl_types.h"
#include "bitrl/extern/nlohmann/json/json.hpp"

#include <stdexcept>

namespace bitrl
{
namespace sensors
{
namespace
{
real_t metres_per_unit(const std::string &unit)
{
    if (unit == "m" || unit == "METERS")
    {
        return 1.0;
    }
    if (unit == "cm")
    {
        return 0.01;
    }
    if (unit == "mm")
    {
        return 0.001;
    }
    return 0.0;
}
} // namespace

real_t UltrasoundMessage::metres() const { return distance * metres_per_unit(unit_str); }

std::optional<UltrasoundMessage> UltrasoundMessage::parse(const std::string &msg)
{
    const auto json = nlohmann::json::parse(msg, nullptr, false);
    if (json.is_discarded() || !json.is_object() || !json.contains("distance") ||
        !json.at("distance").is_number())
    {
        return std::nullopt;
    }

    auto message = UltrasoundMessage();
    message.distance = json.at("distance").get<real_t>();
    if (json.contains("unit"))
    {
        if (!json.at("unit").is_string())
        {
            return std::nullopt;
        }
        message.unit_str = json.at("unit").get<std::string>();
    }
    if (metres_per_unit(message.unit_str) == 0.0)
    {
        return std::nullopt;
    }

    message.source_timestamp = message.received_timestamp;
    if (json.contains("timestamp") && json.at("timestamp").is_string())
    {
        try
        {
            message.source_timestamp = parse_utc_timestamp(json.at("timestamp").get<std::string>());
        }
        catch (const std::runtime_error &)
        {
            return std::nullopt;
        }
    }
    return message;
}
} // namespace sensors
} // namespace bitrl

#include "bitrl/sensors/messages/odometry.h"
#include "bitrl/extern/nlohmann/json/json.hpp"

#include <stdexcept>

namespace bitrl
{
namespace sensors
{
std::optional<OdometryMessage> OdometryMessage::parse(const std::string &msg)
{
    const auto json = nlohmann::json::parse(msg, nullptr, false);
    if (json.is_discarded() || !json.is_object())
    {
        return std::nullopt;
    }
    for (const char *key : {"x", "y", "yaw"})
    {
        if (!json.contains(key) || !json.at(key).is_number())
        {
            return std::nullopt;
        }
    }

    OdometryMessage message;
    message.x = json.at("x").get<real_t>();
    message.y = json.at("y").get<real_t>();
    message.yaw = json.at("yaw").get<real_t>();
    message.source_timestamp = message.received_timestamp;
    if (json.contains("timestamp"))
    {
        if (!json.at("timestamp").is_string())
        {
            return std::nullopt;
        }
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

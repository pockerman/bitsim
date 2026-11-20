#include "bitsim/sensors/messages/infrared.h"
#include "bitsim/extern/nlohmann/json/json.hpp"

#include <algorithm>
#include <stdexcept>

namespace bitsim
{
namespace sensors
{

std::optional<InfraredMessage> InfraredMessage::parse(const std::string &msg)
{
    const auto json = nlohmann::json::parse(msg, nullptr, false);
    if (json.is_discarded() || !json.is_object() || !json.contains("values") ||
        !json.at("values").is_array() || json.at("values").empty())
    {
        return std::nullopt;
    }

    InfraredMessage message;
    for (const auto &value : json.at("values"))
    {
        if (!value.is_number())
        {
            return std::nullopt;
        }
        message.values.push_back(std::clamp(value.get<real_t>(), 0.0, 1.0));
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
} // namespace bitsim

#include "bitsim/sensors/messages/imu.h"
#include "bitsim/extern/nlohmann/json/json.hpp"

#include <stdexcept>

namespace bitsim
{
namespace sensors
{
namespace
{
bool read_vector(const nlohmann::json &j, const char *key, std::array<real_t, 3> &out)
{
    if (!j.contains(key) || !j.at(key).is_array() || j.at(key).size() != 3)
    {
        return false;
    }
    for (std::size_t i = 0; i < 3; ++i)
    {
        if (!j.at(key).at(i).is_number())
        {
            return false;
        }
        out[i] = j.at(key).at(i).get<real_t>();
    }
    return true;
}
} // namespace

std::optional<IMUMessage> IMUMessage::parse(const std::string &msg)
{
    const auto json = nlohmann::json::parse(msg, nullptr, false);
    if (json.is_discarded() || !json.is_object())
    {
        return std::nullopt;
    }

    IMUMessage message;
    if (!read_vector(json, "accel", message.accel) || !read_vector(json, "gyro", message.gyro))
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
} // namespace bitsim

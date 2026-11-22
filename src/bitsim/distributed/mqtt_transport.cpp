#include "bitsim/distributed/mqtt_transport.h"

#ifdef BITSIM_MQTT

namespace bitsim
{
namespace distributed
{

MQTT_Transport::MQTT_Transport(const std::string &broker, const std::string &world)
    : client_(broker, topic(world))
{
    client_.connect();
}

void MQTT_Transport::publish(const std::string &bytes) { client_.publish(bytes, 0, false); }

std::optional<std::string> MQTT_Transport::receive(std::chrono::milliseconds timeout)
{
    return client_.poll(timeout);
}

} // namespace distributed
} // namespace bitsim

#endif

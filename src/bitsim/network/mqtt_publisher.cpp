#include "bitsim/network/mqtt_publisher.h"

#include <mqtt/message.h>
#include <stdexcept>

namespace bitsim
{
namespace network
{

MqttPublisher::MqttPublisher(const std::string &server, const std::string &topic)
    : server_(server), topic_(topic), cli_(server, "")
{
}

MqttPublisher::~MqttPublisher()
{
    try
    {
        cli_.disconnect()->wait();
    }
    catch (...)
    {
    }
}

void MqttPublisher::connect()
{
    mqtt::connect_options options;
    options.set_clean_session(true);
    cli_.connect(options)->wait();
}

void MqttPublisher::publish(const std::string &payload, int_t qos, bool retained)
{
    if (!cli_.is_connected())
    {
        throw std::runtime_error("MQTT client not connected");
    }

    auto msg = mqtt::make_message(topic_, payload);
    msg->set_qos(qos);
    msg->set_retained(retained);
    cli_.publish(msg);
}

void MqttPublisher::publish(const nlohmann::json &payload, int_t qos, bool retained)
{
    publish(payload.dump(), qos, retained);
}

} // namespace network
} // namespace bitsim

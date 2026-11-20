#include "bitsim/bitsim_config.h"

#ifdef BITSIM_MQTT

#include "bitsim/sensors/backends/mqtt_sensor_channel.h"

#include <stdexcept>

namespace bitsim
{
namespace sensors::backends
{

void MQTT_SensorChannel::Config::read(const nlohmann::json &j)
{
    broker = j.value("broker", broker);
    topic = j.value("topic", topic);
    timeout = j.value("timeout", timeout);
}

MQTT_SensorChannel::MQTT_SensorChannel(network::MqttSubscriber &subscriber)
    : borrowed_subscriber_(&subscriber)
{
    config_.topic = subscriber.topic();
}

network::MqttSubscriber *MQTT_SensorChannel::subscriber()
{
    return borrowed_subscriber_ ? borrowed_subscriber_ : own_subscriber_.get();
}

void MQTT_SensorChannel::connect()
{
    if (!borrowed_subscriber_ && !own_subscriber_)
    {
        if (config_.topic.empty())
        {
            throw std::logic_error("The MQTT sensor channel has no topic");
        }
        own_subscriber_ = std::make_unique<network::MqttSubscriber>(config_.broker, config_.topic);
    }

    auto *sub = subscriber();
    if (!sub->is_connected())
    {
        sub->connect();
    }
}

bool MQTT_SensorChannel::is_connected() const
{
    const auto *sub = borrowed_subscriber_ ? borrowed_subscriber_ : own_subscriber_.get();
    return sub && sub->is_connected();
}

void MQTT_SensorChannel::mark_received()
{
    last_received_ = Clock::now();
    ++n_received_;
}

bool MQTT_SensorChannel::is_alive() const
{
    return last_received_ &&
           std::chrono::duration<real_t>(Clock::now() - *last_received_).count() < config_.timeout;
}

} // namespace sensors::backends
} // namespace bitsim

#endif

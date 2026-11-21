#include "bitsim/bitsim_config.h"

#ifdef BITSIM_MQTT
#include "bitsim/sensors/backends/mqtt_ultrasound_backend.h"

#ifdef BITSIM_LOG
#include "bitsim/utils/logger.h"
#endif

namespace bitsim
{
namespace sensors::backends
{
const std::string MQTT_UltrasonicBackend::BACKEND_TYPE = "MQTT";

MQTT_UltrasonicBackend::MQTT_UltrasonicBackend()
    : RangeSensorBackendBase(MQTT_UltrasonicBackend::BACKEND_TYPE)
{
}

MQTT_UltrasonicBackend::MQTT_UltrasonicBackend(const MQTT_SensorChannel::Config &config)
    : RangeSensorBackendBase(MQTT_UltrasonicBackend::BACKEND_TYPE)
{
    channel_.set_config(config);
}

MQTT_UltrasonicBackend::MQTT_UltrasonicBackend(network::MqttSubscriber &subscriber)
    : RangeSensorBackendBase(MQTT_UltrasonicBackend::BACKEND_TYPE), channel_(subscriber)
{
}

void MQTT_UltrasonicBackend::load(const nlohmann::json &j)
{
    RangeSensorBackendBase::load(j);

    auto config = channel_.config();
    config.read(j);
    channel_.set_config(config);

#ifdef BITSIM_LOG
    BITSIM_LOG_INFO << "Loaded sensor backend: MQTT_UltrasonicBackend";
#endif
}

bool MQTT_UltrasonicBackend::receive(const std::string &text)
{
    if (!accept_(text))
    {
        return false;
    }
    channel_.mark_received();
    return true;
}

bool MQTT_UltrasonicBackend::accept_(const std::string &text)
{
    const auto message = UltrasoundMessage::parse(text);
    if (!message)
    {
#ifdef BITSIM_LOG
        BITSIM_LOG_WARN << "Ignoring an invalid ultrasound message: " << text;
#endif
        return false;
    }
    last_message_ = *message;
    has_message_ = true;
    return true;
}

std::vector<real_t> MQTT_UltrasonicBackend::read_values()
{
    channel_.read([this](const std::string &text) { return accept_(text); });

    if (!has_message_)
    {
        return {this->max_distance()};
    }
    return {this->clamp_distance(last_message_.metres())};
}

} // namespace sensors::backends
} // namespace bitsim

#endif

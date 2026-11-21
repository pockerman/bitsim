#include "bitsim/bitsim_config.h"

#ifdef BITSIM_MQTT

#include "bitsim/sensors/backends/mqtt_infrared_backend.h"

#ifdef BITSIM_LOG
#include "bitsim/utils/logger.h"
#endif

namespace bitsim
{
namespace sensors::backends
{

const std::string MQTT_InfraredBackend::BACKEND_TYPE = "MQTT";

MQTT_InfraredBackend::MQTT_InfraredBackend() : InfraredBackendBase(BACKEND_TYPE) {}

MQTT_InfraredBackend::MQTT_InfraredBackend(const MQTT_SensorChannel::Config &config)
    : InfraredBackendBase(BACKEND_TYPE)
{
    channel_.set_config(config);
}

MQTT_InfraredBackend::MQTT_InfraredBackend(network::MqttSubscriber &subscriber)
    : InfraredBackendBase(BACKEND_TYPE), channel_(subscriber)
{
}

void MQTT_InfraredBackend::load(const nlohmann::json &j)
{
    InfraredBackendBase::load(j);

    auto config = channel_.config();
    config.read(j);
    channel_.set_config(config);
}

bool MQTT_InfraredBackend::receive(const std::string &text)
{
    if (!accept_(text))
    {
        return false;
    }
    channel_.mark_received();
    return true;
}

bool MQTT_InfraredBackend::accept_(const std::string &text)
{
    const auto message = InfraredMessage::parse(text);
    if (!message || message->values.size() != n_elements())
    {
#ifdef BITSIM_LOG
        BITSIM_LOG_WARN << "Ignoring an invalid infrared message: " << text;
#endif
        return false;
    }
    last_message_ = *message;
    has_message_ = true;
    return true;
}

std::vector<real_t> MQTT_InfraredBackend::read_values()
{
    channel_.read([this](const std::string &text) { return accept_(text); });

    if (!has_message_)
    {
        return std::vector<real_t>(n_elements(), 1.0);
    }
    return last_message_.values;
}

} // namespace sensors::backends
} // namespace bitsim

#endif

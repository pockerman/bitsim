#include "bitsim/bitsim_config.h"

#ifdef BITSIM_MQTT

#include "bitsim/sensors/backends/mqtt_imu_backend.h"

#ifdef BITSIM_LOG
#include "bitsim/utils/logger.h"
#endif

namespace bitsim
{
namespace sensors::backends
{

const std::string MQTT_IMUBackend::BACKEND_TYPE = "MQTT";

MQTT_IMUBackend::MQTT_IMUBackend() : IMUBackendBase(BACKEND_TYPE) {}

MQTT_IMUBackend::MQTT_IMUBackend(const MQTT_SensorChannel::Config &config)
    : IMUBackendBase(BACKEND_TYPE)
{
    channel_.set_config(config);
}

MQTT_IMUBackend::MQTT_IMUBackend(network::MqttSubscriber &subscriber)
    : IMUBackendBase(BACKEND_TYPE), channel_(subscriber)
{
}

void MQTT_IMUBackend::load(const nlohmann::json &j)
{
    IMUBackendBase::load(j);

    auto config = channel_.config();
    config.read(j);
    channel_.set_config(config);
}

bool MQTT_IMUBackend::receive(const std::string &text)
{
    if (!accept_(text))
    {
        return false;
    }
    channel_.mark_received();
    return true;
}

bool MQTT_IMUBackend::accept_(const std::string &text)
{
    const auto message = IMUMessage::parse(text);
    if (!message)
    {
#ifdef BITSIM_LOG
        BITSIM_LOG_WARN << "Ignoring an invalid IMU message: " << text;
#endif
        return false;
    }
    last_message_ = *message;
    return true;
}

std::vector<real_t> MQTT_IMUBackend::read_values()
{
    channel_.read([this](const std::string &text) { return accept_(text); });

    const auto &m = last_message_;
    return {m.accel[0], m.accel[1], m.accel[2], m.gyro[0], m.gyro[1], m.gyro[2]};
}

} // namespace sensors::backends
} // namespace bitsim

#endif

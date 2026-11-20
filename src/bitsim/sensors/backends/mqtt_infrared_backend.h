#ifndef MQTT_INFRARED_BACKEND_H
#define MQTT_INFRARED_BACKEND_H

#include "bitsim/bitsim_config.h"

#ifdef BITSIM_MQTT

#include "bitsim/network/mqtt_subscriber.h"
#include "bitsim/sensors/backends/infrared_backend_base.h"
#include "bitsim/sensors/backends/mqtt_sensor_channel.h"
#include "bitsim/sensors/messages/infrared.h"

#include <string>
#include <vector>

namespace bitsim
{
namespace sensors::backends
{

/**
 * @class MQTT_InfraredBackend
 * @ingroup bitsim_sensors_backends
 * @brief Real infrared reflectance elements on a board that publishes their
 * readings over MQTT, as an InfraredMessage with one value per element:
 *
 * @code
 * {"values": [0.91, 0.12, 0.88]}
 * @endcode
 *
 * read_values() never waits for the board. It takes every message that has
 * arrived and reports the latest values. A message with a different number of
 * values than the backend has elements is ignored. Until the first reading
 * arrives every element reads 1, white, so check is_alive() before trusting a
 * reading.
 */
class MQTT_InfraredBackend final : public InfraredBackendBase
{
  public:
    static const std::string BACKEND_TYPE;

    ///
    /// @brief A backend that subscribes on its own, see connect(). Give it a
    /// topic with set_channel_config() or load()
    ///
    MQTT_InfraredBackend();

    explicit MQTT_InfraredBackend(const MQTT_SensorChannel::Config &config);

    ///
    /// @brief A backend that reads a subscriber the application connected.
    /// The subscriber must outlive the backend
    ///
    explicit MQTT_InfraredBackend(network::MqttSubscriber &subscriber);

    ///
    /// @brief Read the settings, see InfraredBackendBase::load(). Also reads
    /// `broker`, `topic` and `timeout` [s], all optional
    ///
    void load(const nlohmann::json &j) override;

    void set_channel_config(const MQTT_SensorChannel::Config &config)
    {
        channel_.set_config(config);
    }

    ///
    /// @brief Connect to the broker, unless the subscriber already is
    ///
    void connect() { channel_.connect(); }

    ///
    /// @brief Take the readings that have arrived
    /// @return The latest value of each element, in [0, 1]
    /// @throws std::logic_error If the backend is not connected
    ///
    std::vector<real_t> read_values() override;

    ///
    /// @brief Take one message, as read_values() does for each message that
    /// arrives. Use it to feed the backend without a broker
    /// @return True if it was a valid InfraredMessage with one value per element
    ///
    bool receive(const std::string &text);

    bool is_alive() const { return channel_.is_alive(); }

    const MQTT_SensorChannel &channel() const noexcept { return channel_; }

    ///
    /// @return The last valid message
    ///
    const InfraredMessage &last_value_read() const noexcept { return last_message_; }

  private:
    bool accept_(const std::string &text);

    MQTT_SensorChannel channel_;
    InfraredMessage last_message_;
    bool has_message_{false};
};

} // namespace sensors::backends
} // namespace bitsim

#endif
#endif // MQTT_INFRARED_BACKEND_H

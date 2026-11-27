#ifndef BITSIM_SENSORS_BACKENDS_MQTT_SENSOR_CHANNEL_H
#define BITSIM_SENSORS_BACKENDS_MQTT_SENSOR_CHANNEL_H

#include "bitsim/bitsim_config.h"

#ifdef BITSIM_MQTT

#include "bitsim/bitsim_types.h"
#include "bitsim/extern/nlohmann/json/json.hpp"
#include "bitsim/network/mqtt_subscriber.h"

#include <chrono>
#include <memory>
#include <optional>
#include <string>

namespace bitsim
{
namespace sensors::backends
{

/**
 * @class MQTT_SensorChannel
 * @ingroup bitsim_sensors_backends
 * @brief The topic a sensor on a board publishes its readings to.
 *
 * The channel either uses a subscriber the application connected, or makes
 * and connects its own from config() when connect() is called. Reading the
 * channel never waits: it hands over every message that has arrived since the
 * last read, so a simulation that reads it every step keeps up with a board
 * that publishes often, and sees the latest reading.
 */
class MQTT_SensorChannel
{
  public:
    using Clock = std::chrono::steady_clock;

    struct Config
    {
        std::string broker{"tcp://localhost:1883"};
        std::string topic;

        ///
        /// @brief No reading for this long means the board is gone [s]
        ///
        real_t timeout{0.5};

        ///
        /// @brief Read the keys broker, topic and timeout, all optional
        ///
        void read(const nlohmann::json &j);
    };

    ///
    /// @brief A channel that makes its own subscriber when connect() is called
    ///
    MQTT_SensorChannel() = default;

    ///
    /// @brief A channel that reads the given subscriber. It must outlive the
    /// channel
    ///
    explicit MQTT_SensorChannel(network::MqttSubscriber &subscriber);

    const Config &config() const noexcept { return config_; }
    void set_config(const Config &config) { config_ = config; }

    ///
    /// @brief Connect to the broker, unless the subscriber already is
    /// @throws std::logic_error If the channel has no topic to subscribe to
    ///
    void connect();

    bool is_connected() const;

    ///
    /// @brief Pass every message that has arrived to receive, oldest first,
    /// without waiting. receive returns true if it accepted the message
    /// @throws std::logic_error If the channel is not connected
    ///
    template <typename Receive> void read(Receive &&receive);

    ///
    /// @brief Record that a valid reading arrived now. read() does it for the
    /// messages receive accepts; call it when feeding messages by hand
    ///
    void mark_received();

    ///
    /// @return True if a reading arrived within the timeout
    ///
    bool is_alive() const;

    uint_t n_received() const noexcept { return n_received_; }
    uint_t n_rejected() const noexcept { return n_rejected_; }

  private:
    network::MqttSubscriber *subscriber();

    Config config_;
    std::unique_ptr<network::MqttSubscriber> own_subscriber_;
    network::MqttSubscriber *borrowed_subscriber_{nullptr};

    std::optional<Clock::time_point> last_received_;
    uint_t n_received_{0};
    uint_t n_rejected_{0};
};

template <typename Receive> void MQTT_SensorChannel::read(Receive &&receive)
{
    auto *sub = subscriber();
    if (!sub || !sub->is_connected())
    {
        throw std::logic_error("The MQTT sensor channel is not connected: call connect()");
    }

    while (const auto text = sub->poll(std::chrono::milliseconds(0)))
    {
        if (receive(*text))
        {
            mark_received();
        }
        else
        {
            ++n_rejected_;
        }
    }
}

} // namespace sensors::backends
} // namespace bitsim

#endif
#endif // BITSIM_SENSORS_BACKENDS_MQTT_SENSOR_CHANNEL_H

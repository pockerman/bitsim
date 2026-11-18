#ifndef MQTT_ULTRASOUND_BACKEND_H
#define MQTT_ULTRASOUND_BACKEND_H

#include "bitrl/bitrl_config.h"

#ifdef BITRL_MQTT

#include "bitrl/bitrl_types.h"
#include "bitrl/network/mqtt_subscriber.h"
#include "bitrl/sensors/backends/mqtt_sensor_channel.h"
#include "bitrl/sensors/backends/range_sensor_backend_base.h"
#include "bitrl/sensors/messages/ultrasound.h"

#include <optional>
#include <string>

namespace bitrl
{
namespace sensors::backends
{

/**
 * @class MQTT_UltrasonicBackend
 * @ingroup bitrl_sensors_backends
 * @brief A real ultrasonic sensor on a board that publishes its readings over
 * MQTT, as an UltrasoundMessage:
 *
 * @code
 * {"distance": 62.0, "unit": "cm"}
 * @endcode
 *
 * read_values() never waits for the board. It takes every message that has
 * arrived and reports the latest distance, in metres. Until the first reading
 * arrives it reports max_distance(), as if nothing were in range, so check
 * is_alive() before trusting a reading: a board that has gone quiet keeps
 * reporting its last distance.
 */
class MQTT_UltrasonicBackend : public RangeSensorBackendBase
{
  public:
    /**
     * @brief The type of the backend
     */
    static const std::string BACKEND_TYPE;

    ///
    /// @brief A backend that subscribes on its own, see connect(). Give it a
    /// topic with set_channel_config() or load()
    ///
    MQTT_UltrasonicBackend();

    ///
    /// @brief A backend that subscribes on its own to the given broker and
    /// topic, see connect()
    ///
    explicit MQTT_UltrasonicBackend(const MQTT_SensorChannel::Config &config);

    ///
    /// @brief A backend that reads a subscriber the application connected.
    /// The subscriber must outlive the backend
    ///
    explicit MQTT_UltrasonicBackend(network::MqttSubscriber &subscriber);

    ///
    /// @brief Read the settings, see RangeSensorBackendBase::load(). Also
    /// reads `broker`, `topic` and `timeout` [s], all optional
    ///
    void load(const nlohmann::json &j) override;

    ///
    /// @brief Set where the readings come from. Call it before connect()
    ///
    void set_channel_config(const MQTT_SensorChannel::Config &config)
    {
        channel_.set_config(config);
    }

    ///
    /// @brief Connect to the broker, unless the subscriber already is
    ///
    void connect() { channel_.connect(); }

    /**
     * @brief Take the readings that have arrived
     * @return The latest distance [m], clamped to [min_distance(), max_distance()]
     * @throws std::logic_error If the backend is not connected
     */
    std::vector<real_t> read_values() override;

    ///
    /// @brief Take one message, as read_values() does for each message that
    /// arrives. Use it to feed the backend without a broker
    /// @return True if it was a valid UltrasoundMessage
    ///
    bool receive(const std::string &text);

    ///
    /// @return True if a reading arrived within the timeout of the channel
    ///
    bool is_alive() const { return channel_.is_alive(); }

    ///
    /// @return The topic the readings come from, and its statistics
    ///
    const MQTT_SensorChannel &channel() const noexcept { return channel_; }

    /**
     * @brief Set the position of the sensor on the robot. The backend does not
     * use it
     * @param pos
     */
    void set_position(const RealColVec3d &pos) { position_ = pos; }

    /**
     * @return A read reference of the position of the sensor on the robot
     */
    const RealColVec3d &position() const { return position_; }

    /**
     * @return A read reference to the last UltrasoundMessage that was read
     */
    const UltrasoundMessage &last_value_read() const { return last_message_; }

  private:
    bool accept_(const std::string &text);

    MQTT_SensorChannel channel_;

    /**
     * @brief The position of the sensor
     */
    RealColVec3d position_{RealColVec3d::Zero()};

    /**
     * @brief The last valid message
     */
    UltrasoundMessage last_message_;
    bool has_message_{false};
};

} // namespace sensors::backends
} // namespace bitrl
#endif
#endif // MQTT_ULTRASOUND_BACKEND_H

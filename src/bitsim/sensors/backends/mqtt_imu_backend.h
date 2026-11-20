#ifndef MQTT_IMU_BACKEND_H
#define MQTT_IMU_BACKEND_H

#include "bitsim/bitsim_config.h"

#ifdef BITSIM_MQTT

#include "bitsim/network/mqtt_subscriber.h"
#include "bitsim/sensors/backends/imu_backend_base.h"
#include "bitsim/sensors/backends/mqtt_sensor_channel.h"
#include "bitsim/sensors/messages/imu.h"

#include <string>
#include <vector>

namespace bitsim
{
namespace sensors::backends
{

/**
 * @class MQTT_IMUBackend
 * @ingroup bitsim_sensors_backends
 * @brief A real IMU on a board that publishes its readings over MQTT, as an
 * IMUMessage:
 *
 * @code
 * {"accel": [0.12, -0.03, 9.79], "gyro": [0.001, -0.002, 0.35]}
 * @endcode
 *
 * read_values() never waits for the board. It takes every message that has
 * arrived and reports the latest. Until the first reading arrives it reports
 * zeros, so check is_alive() before trusting a reading.
 */
class MQTT_IMUBackend final : public IMUBackendBase
{
  public:
    static const std::string BACKEND_TYPE;

    ///
    /// @brief A backend that subscribes on its own, see connect(). Give it a
    /// topic with set_channel_config() or load()
    ///
    MQTT_IMUBackend();

    explicit MQTT_IMUBackend(const MQTT_SensorChannel::Config &config);

    ///
    /// @brief A backend that reads a subscriber the application connected.
    /// The subscriber must outlive the backend
    ///
    explicit MQTT_IMUBackend(network::MqttSubscriber &subscriber);

    ///
    /// @brief Read the settings, see IMUBackendBase::load(). Also reads
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
    /// @return The latest specific force [m/s^2] and angular velocity [rad/s]
    /// @throws std::logic_error If the backend is not connected
    ///
    std::vector<real_t> read_values() override;

    ///
    /// @brief Take one message, as read_values() does for each message that
    /// arrives. Use it to feed the backend without a broker
    /// @return True if it was a valid IMUMessage
    ///
    bool receive(const std::string &text);

    bool is_alive() const { return channel_.is_alive(); }

    const MQTT_SensorChannel &channel() const noexcept { return channel_; }

    const IMUMessage &last_value_read() const noexcept { return last_message_; }

  private:
    bool accept_(const std::string &text);

    MQTT_SensorChannel channel_;
    IMUMessage last_message_;
};

} // namespace sensors::backends
} // namespace bitsim

#endif
#endif // MQTT_IMU_BACKEND_H

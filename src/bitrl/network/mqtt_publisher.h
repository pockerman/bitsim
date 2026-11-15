#ifndef MQTT_PUBLISHER_H
#define MQTT_PUBLISHER_H

#include "bitrl/bitrl_types.h"
#include "bitrl/extern/nlohmann/json/json.hpp"

#include <mqtt/async_client.h>
#include <string>

namespace bitrl
{
namespace network
{
///
/// A thin wrapper over mqtt::async_client that only publishes to one topic.
/// Unlike MqttSubscriber it does not subscribe, so it does not receive its
/// own messages back
///
class MqttPublisher
{
  public:
    MqttPublisher(const std::string &server, const std::string &topic);
    ~MqttPublisher();

    std::string topic() const noexcept { return topic_; }

    bool is_connected() const noexcept { return cli_.is_connected(); }

    void connect();

    ///  Publish the payload to the topic
    /// @param payload
    /// @param qos
    /// @param retained
    ///
    void publish(const std::string &payload, int_t qos = 0, bool retained = false);

    ///  Publish the JSON payload
    /// @param payload
    /// @param qos
    /// @param retained
    void publish(const nlohmann::json &payload, int_t qos = 0, bool retained = false);

  private:
    const std::string server_;
    const std::string topic_;

    mqtt::async_client cli_;
};
} // namespace network
} // namespace bitrl

#endif // MQTT_PUBLISHER_H

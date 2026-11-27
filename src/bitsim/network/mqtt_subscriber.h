//
// Created by alex on 11/23/25.
//

#ifndef BITSIM_NETWORK_MQTT_SUBSCRIBER_H
#define BITSIM_NETWORK_MQTT_SUBSCRIBER_H

#include "bitsim/bitsim_types.h"
#include "bitsim/extern/nlohmann/json/json.hpp"
#include "bitsim/utils/concurrency/concurrent_queue.h"

#include <mqtt/async_client.h>
#include <optional>
#include <string>

namespace bitsim
{
namespace network
{
///
/// A thin wrapper over mqtt::async_client
///
class MqttSubscriber : public virtual mqtt::callback
{
  public:
    MqttSubscriber(const std::string &server, const std::string &topic);
    ~MqttSubscriber() override;

    std::string topic() const noexcept { return topic_; }

    bool is_connected() const noexcept { return cli_.is_connected(); }

    void connect();
    std::optional<std::string>
    poll(std::chrono::milliseconds timeout = std::chrono::milliseconds(1000));

    std::optional<std::string>
    read(std::chrono::milliseconds timeout = std::chrono::milliseconds::zero());

    void message_arrived(mqtt::const_message_ptr msg) override;

    ///  Publish the payload to the topic that this subscriber listens to
    /// @param payload
    /// @param qos
    /// @param retained
    ///
    void publish(const std::string &payload, int_t qos = 1, bool retained = false);

    ///  Publish the JSON payload
    /// @param payload
    /// @param qos
    /// @param retained
    void publish(const nlohmann::json &payload, int_t qos = 1, bool retained = false);

  private:
    const std::string server_;
    const std::string topic_;

    mqtt::async_client cli_;
    mqtt::connect_options conn_opts_;

    // the client's thread pushes the messages, poll() and read() take them
    utils::concurrency::ConcurrentQueue<std::string> queue_;
};
} // namespace network
} // namespace bitsim

#endif // BITSIM_NETWORK_MQTT_SUBSCRIBER_H

//
// Created by alex on 11/23/25.
//

#include "bitsim/network/mqtt_subscriber.h"

#include <mqtt/message.h>
#include <optional>

namespace bitsim
{
namespace network
{

MqttSubscriber::MqttSubscriber(const std::string &server, const std::string &topic)
    : server_(server), topic_(topic), cli_(server, "")
{
}

MqttSubscriber::~MqttSubscriber()
{
    try
    {
        cli_.stop_consuming();
        cli_.disconnect()->wait();
    }
    catch (...)
    {
    }
}

void MqttSubscriber::connect()
{
    conn_opts_.set_clean_session(true);
    cli_.connect(conn_opts_)->wait();
    cli_.set_callback(*this);
    cli_.start_consuming();
    cli_.subscribe(topic_, 1)->wait();
}

void MqttSubscriber::message_arrived(mqtt::const_message_ptr msg) { queue_.push(msg->to_string()); }

std::optional<std::string> MqttSubscriber::poll(std::chrono::milliseconds timeout)
{
    if (!cli_.is_connected())
    {
        throw std::runtime_error("MQTT client not connected");
    }
    return queue_.pop_for(timeout);
}

std::optional<std::string> MqttSubscriber::read(std::chrono::milliseconds timeout)
{
    if (!cli_.is_connected())
    {
        throw std::runtime_error("MQTT client not connected");
    }
    // a zero timeout waits for as long as it takes
    return timeout == std::chrono::milliseconds::zero() ? queue_.pop() : queue_.pop_for(timeout);
}

void MqttSubscriber::publish(const std::string &payload, int_t qos, bool retained)
{
    if (!cli_.is_connected())
    {
        throw std::runtime_error("MQTT client not connected");
    }

    auto msg = mqtt::make_message(topic_, payload);
    msg->set_qos(qos);
    msg->set_retained(retained);
    cli_.publish(msg);
}

void MqttSubscriber::publish(const nlohmann::json &payload, int_t qos, bool retained)
{

    // Serialize JSON to string
    std::string msg_str = payload.dump();
    publish(msg_str, qos, retained);
}

} // namespace network
} // namespace bitsim

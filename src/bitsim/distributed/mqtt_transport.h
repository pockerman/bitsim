#ifndef BITSIM_DISTRIBUTED_MQTT_TRANSPORT_H
#define BITSIM_DISTRIBUTED_MQTT_TRANSPORT_H

#include "bitsim/bitsim_config.h"

#ifdef BITSIM_MQTT

#include "bitsim/distributed/transport.h"
#include "bitsim/network/mqtt_subscriber.h"

#include <string>

namespace bitsim
{
namespace distributed
{

///
/// \brief Participants in several processes, on one machine or several, that meet on an MQTT
/// broker.
///
/// Every participant of a world publishes to and subscribes to the topic
/// "bitsim/world/<world>", so it also receives its own messages, which Synchroniser ignores.
/// The messages are published with quality of service 0: a lost state is replaced by the next,
/// or sent again by the synchroniser in lockstep. A robot on the network, a Raspberry Pi for
/// example, joins the world by publishing StateMessage::encode() to the same topic.
///
class MQTT_Transport final : public Transport
{
  public:
    ///
    /// \param broker the address of the broker, "tcp://localhost:1883" for example
    /// \param world the name of the world; participants of different worlds do not meet
    ///
    MQTT_Transport(const std::string &broker, const std::string &world);

    static std::string topic(const std::string &world) { return "bitsim/world/" + world; }

    void publish(const std::string &bytes) override;

    std::optional<std::string> receive(std::chrono::milliseconds timeout) override;

  private:
    network::MqttSubscriber client_;
};

} // namespace distributed
} // namespace bitsim

#endif
#endif // BITSIM_DISTRIBUTED_MQTT_TRANSPORT_H

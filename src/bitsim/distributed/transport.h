#ifndef TRANSPORT_H
#define TRANSPORT_H

#include "bitsim/bitsim_types.h"
#include "bitsim/utils/concurrency/concurrent_queue.h"

#include <boost/noncopyable.hpp>

#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <random>
#include <string>
#include <vector>

namespace bitsim
{
namespace distributed
{

///
/// \brief How the participants of a distributed simulation reach each other: everything one
/// publishes, every other receives.
///
/// The messages are bytes; the synchroniser sends StateMessage::encode(). A transport may lose
/// messages, MQTT does when nobody listens yet, and it may give a participant its own messages
/// back, MQTT does that too; Synchroniser copes with both. Implementations:
/// InProcessBus, for participants that are threads of one process, MQTT_Transport, for
/// processes on one or several machines, and LossyTransport, which adds delay and losses to
/// another transport for testing.
///
class Transport : private boost::noncopyable
{
  public:
    virtual ~Transport() = default;

    ///
    /// \brief Send bytes to every other participant
    ///
    virtual void publish(const std::string &bytes) = 0;

    ///
    /// \brief The next message from another participant, waiting at most timeout
    ///
    virtual std::optional<std::string> receive(std::chrono::milliseconds timeout) = 0;
};

///
/// \brief Participants that are threads of one process, joined by queues.
///
/// connect() gives every participant its own Transport; what one publishes is queued for each of
/// the others that is connected at that time, so, like MQTT, a late participant misses what was
/// sent before it connected. The bus must outlive its transports.
///
class InProcessBus : private boost::noncopyable
{
  public:
    ///
    /// \brief A transport for a new participant
    ///
    std::unique_ptr<Transport> connect();

  private:
    class Endpoint;
    void publish_(const Endpoint *from, const std::string &bytes);

    std::mutex mutex_;
    std::vector<Endpoint *> endpoints_;
};

///
/// \brief A transport that delays and loses some of the messages of another, as a network would.
///
/// Every received message is held back for latency plus a random jitter, and dropped with
/// probability loss, both drawn from a generator seeded with seed. Messages can overtake each
/// other when the jitter is larger than the time between them. It is for testing how a
/// simulation copes, without a network.
///
class LossyTransport final : public Transport
{
  public:
    struct Config
    {
        real_t latency{0.0}; ///< [s]
        real_t jitter{0.0};  ///< [s], the standard deviation of a delay added to latency
        real_t loss{0.0};    ///< the probability that a message is lost
        uint_t seed{42};
    };

    LossyTransport(std::unique_ptr<Transport> inner, const Config &config);

    void publish(const std::string &bytes) override { inner_->publish(bytes); }

    std::optional<std::string> receive(std::chrono::milliseconds timeout) override;

    uint_t n_lost() const noexcept { return n_lost_; }

  private:
    using clock_type = std::chrono::steady_clock;
    struct Held
    {
        clock_type::time_point release;
        uint_t order;
        std::string bytes;
        bool operator>(const Held &other) const
        {
            return release > other.release || (release == other.release && order > other.order);
        }
    };

    std::unique_ptr<Transport> inner_;
    Config config_;
    std::mt19937 rng_;
    std::priority_queue<Held, std::vector<Held>, std::greater<Held>> held_;
    uint_t n_held_{0};
    uint_t n_lost_{0};
};

} // namespace distributed
} // namespace bitsim

#endif // TRANSPORT_H

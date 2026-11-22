#include "bitsim/distributed/transport.h"

#include <algorithm>

namespace bitsim
{
namespace distributed
{

class InProcessBus::Endpoint final : public Transport
{
  public:
    explicit Endpoint(InProcessBus &bus) : bus_(bus) {}

    ~Endpoint() override
    {
        std::lock_guard<std::mutex> lock(bus_.mutex_);
        auto &e = bus_.endpoints_;
        e.erase(std::remove(e.begin(), e.end(), this), e.end());
    }

    void publish(const std::string &bytes) override { bus_.publish_(this, bytes); }

    std::optional<std::string> receive(std::chrono::milliseconds timeout) override
    {
        return inbox_.pop_for(timeout);
    }

    void deliver(const std::string &bytes) { inbox_.push(bytes); }

  private:
    InProcessBus &bus_;
    utils::concurrency::ConcurrentQueue<std::string> inbox_;
};

std::unique_ptr<Transport> InProcessBus::connect()
{
    auto endpoint = std::make_unique<Endpoint>(*this);
    std::lock_guard<std::mutex> lock(mutex_);
    endpoints_.push_back(endpoint.get());
    return endpoint;
}

void InProcessBus::publish_(const Endpoint *from, const std::string &bytes)
{
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto *endpoint : endpoints_)
    {
        if (endpoint != from)
        {
            endpoint->deliver(bytes);
        }
    }
}

LossyTransport::LossyTransport(std::unique_ptr<Transport> inner, const Config &config)
    : inner_(std::move(inner)), config_(config),
      rng_(static_cast<std::mt19937::result_type>(config.seed))
{
}

std::optional<std::string> LossyTransport::receive(std::chrono::milliseconds timeout)
{
    const auto deadline = clock_type::now() + timeout;
    std::uniform_real_distribution<real_t> uniform(0.0, 1.0);
    std::normal_distribution<real_t> jitter(0.0, config_.jitter > 0.0 ? config_.jitter : 1.0);

    // hold a message for its delay, or lose it
    auto hold = [&](std::string bytes)
    {
        if (uniform(rng_) < config_.loss)
        {
            ++n_lost_;
            return;
        }
        const real_t delay =
            std::max(0.0, config_.latency + (config_.jitter > 0.0 ? jitter(rng_) : 0.0));
        held_.push({clock_type::now() + std::chrono::duration_cast<clock_type::duration>(
                                            std::chrono::duration<real_t>(delay)),
                    n_held_++, std::move(bytes)});
    };

    while (true)
    {
        // take everything that has arrived
        while (auto bytes = inner_->receive(std::chrono::milliseconds(0)))
        {
            hold(std::move(*bytes));
        }

        const auto now = clock_type::now();
        if (!held_.empty() && held_.top().release <= now)
        {
            auto bytes = held_.top().bytes;
            held_.pop();
            return bytes;
        }
        if (now >= deadline)
        {
            return std::nullopt;
        }

        // wait for a new message, but not past the release of a held one or the deadline
        auto until = deadline;
        if (!held_.empty())
        {
            until = std::min(until, held_.top().release);
        }
        const auto wait = std::chrono::ceil<std::chrono::milliseconds>(until - now);
        if (auto bytes = inner_->receive(wait))
        {
            hold(std::move(*bytes));
        }
    }
}

} // namespace distributed
} // namespace bitsim

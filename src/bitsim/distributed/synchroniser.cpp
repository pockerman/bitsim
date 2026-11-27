#include "bitsim/distributed/synchroniser.h"

#include <algorithm>
#include <stdexcept>
#include <thread>

namespace bitsim
{
namespace distributed
{

namespace
{
template <typename Duration> real_t seconds(Duration d)
{
    return std::chrono::duration<real_t>(d).count();
}
} // namespace

Synchroniser::Synchroniser(Transport &transport, const std::string &participant,
                           const SynchroniserConfig &config)
    : transport_(transport), participant_(participant), config_(config)
{
    if (config_.sync_period <= 0.0 || config_.real_time_factor <= 0.0)
    {
        throw std::invalid_argument("Synchroniser: sync_period and real_time_factor must be "
                                    "positive");
    }
    if (std::find(config_.peers.begin(), config_.peers.end(), participant_) != config_.peers.end())
    {
        throw std::invalid_argument("Synchroniser: " + participant_ + " is among its own peers");
    }
}

Synchroniser::~Synchroniser()
{
    try
    {
        leave();
    }
    catch (...)
    {
    }
}

void Synchroniser::send_(const StateMessage &message)
{
    transport_.publish(message.encode());
    ++n_sent_;
}

void Synchroniser::handle_(const std::string &bytes)
{
    StateMessage message;
    try
    {
        message = StateMessage::decode(bytes);
    }
    catch (const std::invalid_argument &)
    {
        return; // not for us
    }
    if (message.participant == participant_)
    {
        return; // our own, given back by the transport
    }
    ++n_received_;

    if (message.kind == StateMessage::Kind::LEAVE)
    {
        left_at_[message.participant] = message.step;
    }

    if (config_.mode == SyncMode::LOCKSTEP)
    {
        // steps already done are of no use; a resent step is already there
        if (!current_ || message.step + 1 >= current_->step)
        {
            pending_[message.participant].try_emplace(message.step, std::move(message));
        }
    }
    else
    {
        auto itr = latest_.find(message.participant);
        // with jitter, an older state can arrive after a newer one
        if (itr == latest_.end() || message.stamp >= itr->second.stamp ||
            message.kind == StateMessage::Kind::LEAVE)
        {
            latest_[message.participant] = std::move(message);
        }
    }
}

void Synchroniser::drain_()
{
    while (auto bytes = transport_.receive(std::chrono::milliseconds(0)))
    {
        handle_(*bytes);
    }
}

const std::vector<PeerState> &Synchroniser::exchange(uint_t step,
                                                     const std::vector<BodyState> &bodies,
                                                     const std::vector<real_t> &values)
{
    if (has_left_)
    {
        throw std::logic_error("Synchroniser: exchange() after leave()");
    }
    if (current_ && step <= current_->step)
    {
        throw std::invalid_argument("Synchroniser: the steps must increase");
    }

    if (config_.mode == SyncMode::WALL_CLOCK)
    {
        // keep pace with the wall clock before sending, so that the stamp is the time of the step
        const auto now = steady::now();
        if (!start_)
        {
            start_ =
                now - std::chrono::duration_cast<steady::duration>(
                          std::chrono::duration<real_t>(time_of(step) / config_.real_time_factor));
        }
        const auto due =
            *start_ + std::chrono::duration_cast<steady::duration>(
                          std::chrono::duration<real_t>(time_of(step) / config_.real_time_factor));
        if (now < due)
        {
            std::this_thread::sleep_until(due);
        }
        else
        {
            const real_t late = seconds(now - due);
            max_wait_ = std::max(max_wait_, late);
            if (late > config_.sync_period / config_.real_time_factor)
            {
                ++n_overruns_;
            }
        }
    }

    StateMessage message;
    message.kind = StateMessage::Kind::STATE;
    message.participant = participant_;
    message.step = step;
    message.time = time_of(step);
    message.stamp = wall_time_now();
    message.bodies = bodies;
    message.values = values;
    previous_ = std::move(current_);
    current_ = std::move(message);
    send_(*current_);

    return config_.mode == SyncMode::LOCKSTEP ? lockstep_(step) : wall_clock_(step);
}

const std::vector<PeerState> &Synchroniser::lockstep_(uint_t step)
{
    const auto start = steady::now();
    auto last_send = start;

    auto missing = [&]()
    {
        std::vector<std::string> names;
        for (const auto &peer : config_.peers)
        {
            const auto p = pending_.find(peer);
            if (p != pending_.end() && p->second.contains(step))
            {
                continue;
            }
            const auto left = left_at_.find(peer);
            if (left != left_at_.end() && left->second < step)
            {
                continue; // it stopped before this step
            }
            names.push_back(peer);
        }
        return names;
    };

    drain_();
    auto waiting_for = missing();
    while (!waiting_for.empty())
    {
        const auto now = steady::now();
        if (seconds(now - start) > config_.timeout)
        {
            std::string names;
            for (const auto &n : waiting_for)
            {
                names += (names.empty() ? "" : ", ") + n;
            }
            throw std::runtime_error("Synchroniser " + participant_ + ": waited " +
                                     std::to_string(config_.timeout) + " s for step " +
                                     std::to_string(step) + " of " + names);
        }
        if (seconds(now - last_send) >= config_.resend_period)
        {
            if (previous_)
            {
                send_(*previous_);
            }
            send_(*current_);
            ++n_resent_;
            last_send = now;
        }
        const auto wait = std::chrono::ceil<std::chrono::milliseconds>(
            std::chrono::duration<real_t>(config_.resend_period) - (now - last_send));
        if (auto bytes = transport_.receive(std::max(wait, std::chrono::milliseconds(1))))
        {
            handle_(*bytes);
            drain_();
        }
        waiting_for = missing();
    }
    max_wait_ = std::max(max_wait_, seconds(steady::now() - start));

    peers_.clear();
    for (const auto &peer : config_.peers)
    {
        PeerState state;
        state.participant = peer;
        auto &steps = pending_[peer];
        const auto left = left_at_.find(peer);
        // a peer leaves after the step it sent last, so it has left only for the steps after it
        state.left = left != left_at_.end() && left->second < step;
        // the state of the step, or the last one of a peer that left before it
        auto itr = steps.find(step);
        if (itr == steps.end() && !steps.empty())
        {
            itr = std::prev(steps.upper_bound(step));
        }
        if (itr != steps.end())
        {
            state.step = itr->second.step;
            state.time = itr->second.time;
            state.bodies = itr->second.bodies;
            state.values = itr->second.values;
        }
        // keep the last state, for a peer that leaves, and the steps to come
        if (!steps.empty())
        {
            const auto keep = steps.upper_bound(step);
            if (keep != steps.begin())
            {
                steps.erase(steps.begin(), std::prev(keep));
            }
        }
        peers_.push_back(std::move(state));
    }
    std::sort(peers_.begin(), peers_.end(),
              [](const PeerState &a, const PeerState &b) { return a.participant < b.participant; });
    return peers_;
}

const std::vector<PeerState> &Synchroniser::wall_clock_(uint_t /*step*/)
{
    drain_();
    const real_t now = wall_time_now();

    peers_.clear();
    for (const auto &[name, message] : latest_)
    {
        PeerState state;
        state.participant = name;
        state.step = message.step;
        state.time = message.time;
        state.age = std::max(0.0, now - message.stamp);
        state.left = message.kind == StateMessage::Kind::LEAVE;
        state.stale = !state.left && state.age > config_.stale_after;
        state.values = message.values;
        // a stale state is not moved further than stale_after: the guess would only get worse
        const real_t ahead = std::min(state.age, config_.stale_after);
        for (const auto &body : message.bodies)
        {
            state.bodies.push_back(config_.extrapolate && !state.left ? body.extrapolated(ahead)
                                                                      : body);
        }
        peers_.push_back(std::move(state));
    }
    // the peers we expect but have not heard from
    for (const auto &peer : config_.peers)
    {
        if (!latest_.contains(peer))
        {
            PeerState state;
            state.participant = peer;
            state.stale = true;
            peers_.push_back(std::move(state));
        }
    }
    std::sort(peers_.begin(), peers_.end(),
              [](const PeerState &a, const PeerState &b) { return a.participant < b.participant; });
    return peers_;
}

void Synchroniser::leave()
{
    if (has_left_)
    {
        return;
    }
    has_left_ = true;
    StateMessage message;
    message.kind = StateMessage::Kind::LEAVE;
    message.participant = participant_;
    message.step = current_ ? current_->step : 0;
    message.time = current_ ? current_->time : 0.0;
    message.stamp = wall_time_now();
    if (current_)
    {
        // the last state comes with it, for a peer that missed it
        message.bodies = current_->bodies;
        message.values = current_->values;
    }
    // twice, against a lost message
    send_(message);
    send_(message);
}

} // namespace distributed
} // namespace bitsim

#include "bitsim/bitsim_config.h"

#ifdef BITSIM_MQTT

#include "bitsim/boards/arduino/dc_arduino_board_impl.h"
#include "bitsim/extern/nlohmann/json/json.hpp"

#include <algorithm>
#include <stdexcept>
#include <thread>

namespace bitsim
{
namespace boards::arduino
{
namespace
{
real_t seconds_between(DCArduinoBoardImpl::Clock::time_point from,
                       DCArduinoBoardImpl::Clock::time_point to)
{
    return std::chrono::duration<real_t>(to - from).count();
}
} // namespace

DCArduinoBoardImpl::DCArduinoBoardImpl(const Config &config)
    : config_(config), state_(config.broker, config.state_topic),
      commands_(config.broker, config.command_topic), targets_(config.channels.size(), 0.0),
      states_(config.channels.size())
{
    if (config_.channels.empty())
    {
        throw std::logic_error("DCArduinoBoardImpl needs at least one channel");
    }
    if (config_.wheel_radius <= 0.0)
    {
        throw std::logic_error("DCArduinoBoardImpl needs a positive wheel radius");
    }
}

void DCArduinoBoardImpl::connect()
{
    state_.connect();
    commands_.connect();
}

void DCArduinoBoardImpl::set_target_speed(uint_t channel, real_t speed)
{
    targets_.at(channel) = speed;
}

void DCArduinoBoardImpl::update()
{
    while (const auto text = state_.poll(std::chrono::milliseconds(0)))
    {
        receive_(*text);
    }

    const auto now = Clock::now();
    if (!last_sent_ || seconds_between(*last_sent_, now) >= config_.command_period)
    {
        send_command_(now);
    }
}

std::optional<MotorChannelState> DCArduinoBoardImpl::motor_state(uint_t channel) const
{
    return states_.at(channel);
}

bool DCArduinoBoardImpl::is_alive() const
{
    return last_received_ && seconds_between(*last_received_, Clock::now()) < config_.timeout;
}

bool DCArduinoBoardImpl::wait_for_board(std::chrono::milliseconds timeout)
{
    const auto start = Clock::now();
    while (Clock::now() - start < timeout)
    {
        update();
        if (last_received_)
        {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
}

void DCArduinoBoardImpl::send_command_(Clock::time_point now)
{
    ++seq_;
    nlohmann::json message{{"seq", seq_}};
    for (std::size_t c = 0; c < config_.channels.size(); ++c)
    {
        message[config_.channels[c] + "_speed"] = targets_[c];
    }

    sent_at_[seq_ % sent_at_.size()] = now;
    last_sent_ = now;
    commands_.publish(message);
    ++n_sent_;
}

void DCArduinoBoardImpl::receive_(const std::string &text)
{
    const auto message = nlohmann::json::parse(text, nullptr, false);
    if (message.is_discarded() || !message.is_object())
    {
        return;
    }

    // Only accept a report that has every channel in it
    for (const auto &name : config_.channels)
    {
        if (!message.contains(name + "_distance") || !message.contains(name + "_speed"))
        {
            return;
        }
    }

    const auto now = Clock::now();
    for (std::size_t c = 0; c < config_.channels.size(); ++c)
    {
        const auto &name = config_.channels[c];
        MotorChannelState state;
        state.angle = message[name + "_distance"].get<real_t>() / config_.wheel_radius;
        state.speed = message[name + "_speed"].get<real_t>();
        state.received_at = now;
        states_[c] = state;
    }
    last_received_ = now;
    ++n_received_;

    // The time from sending a command to hearing that the board has it
    if (message.contains("ack"))
    {
        const long ack = message["ack"].get<long>();
        if (ack > last_ack_ && ack <= seq_ && seq_ - ack < static_cast<long>(sent_at_.size()))
        {
            last_ack_ = ack;
            round_trip_ = seconds_between(sent_at_[ack % sent_at_.size()], now);
            round_trip_sum_ += round_trip_;
            round_trip_max_ = std::max(round_trip_max_, round_trip_);
            ++n_round_trips_;
        }
    }
}

} // namespace boards::arduino
} // namespace bitsim

#endif

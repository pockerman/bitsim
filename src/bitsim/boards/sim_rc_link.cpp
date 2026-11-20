#include "bitsim/boards/sim_rc_link.h"

#include <stdexcept>

namespace bitsim
{
namespace boards
{

SIM_RCLink::SIM_RCLink(real_t timeout) : timeout_(timeout)
{
    if (timeout <= 0.0)
    {
        throw std::invalid_argument("SIM_RCLink: the timeout must be positive");
    }
}

void SIM_RCLink::send(const control::RCCommand &command, real_t time)
{
    last_ = command.clamped();
    last_time_ = time;
    heard_ = true;
}

bool SIM_RCLink::in_failsafe(real_t time) const noexcept
{
    return !heard_ || time - last_time_ > timeout_;
}

control::RCCommand SIM_RCLink::receive(real_t time) const
{
    return in_failsafe(time) ? control::failsafe_command() : last_;
}

} // namespace boards
} // namespace bitsim

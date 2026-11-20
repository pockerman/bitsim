#ifndef BITSIM_BOARDS_SIM_RC_LINK_H
#define BITSIM_BOARDS_SIM_RC_LINK_H

#include "bitsim/bitsim_types.h"
#include "bitsim/boards/rc_transmitter_base.h"
#include "bitsim/control/rc_command.h"

#include <string>

namespace bitsim
{
namespace boards
{

/**
 * @class SIM_RCLink
 * @brief The radio link to a simulated quadrotor: the transmitter end sends commands, and the
 * simulated receiver reads the latest one.
 *
 * Like a real receiver, it goes into failsafe when it has heard nothing for longer than the
 * timeout, or before the first command: it then gives control::failsafe_command(), which
 * disarms the drone so that its motors stop.
 */
class SIM_RCLink final : public RCTransmitterBase
{
  public:
    ///
    /// @param timeout How long the receiver waits for a command before it goes into failsafe [s]
    ///
    explicit SIM_RCLink(real_t timeout = 0.5);

    void send(const control::RCCommand &command, real_t time) override;

    std::string backend_type_str() const override { return "SIMULATION"; }

    ///
    /// @brief What the receiver passes to the flight controller at time [s]: the last command,
    /// clamped, or the failsafe command
    ///
    control::RCCommand receive(real_t time) const;

    ///
    /// @brief True when the receiver has heard nothing for longer than the timeout at time [s]
    ///
    bool in_failsafe(real_t time) const noexcept;

    real_t timeout() const noexcept { return timeout_; }

  private:
    real_t timeout_;
    control::RCCommand last_{};
    real_t last_time_{0.0};
    bool heard_{false};
};

} // namespace boards
} // namespace bitsim

#endif // BITSIM_BOARDS_SIM_RC_LINK_H

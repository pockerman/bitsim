#ifndef BITSIM_BOARDS_RC_TRANSMITTER_BASE_H
#define BITSIM_BOARDS_RC_TRANSMITTER_BASE_H

#include "bitsim/bitsim_types.h"
#include "bitsim/control/rc_command.h"

#include <string>

namespace bitsim
{
namespace boards
{

/**
 * @class RCTransmitterBase
 * @brief Sends the commands of a radio transmitter to a quadrotor: the laptop takes the place of
 * the hand-held transmitter.
 *
 * A backend decides where the commands go. SIM_RCLink delivers them to a simulated quadrotor; a
 * backend for real hardware would send them to a board with a radio that speaks the drone's
 * protocol. The application sends a command at least as often as the receiver's failsafe
 * timeout, or the drone disarms.
 */
class RCTransmitterBase
{
  public:
    virtual ~RCTransmitterBase() = default;

    ///
    /// @brief Send a command at time [s], the clock of the application
    ///
    virtual void send(const control::RCCommand &command, real_t time) = 0;

    ///
    /// @brief The name of the backend, such as SIMULATION
    ///
    virtual std::string backend_type_str() const = 0;
};

} // namespace boards
} // namespace bitsim

#endif // BITSIM_BOARDS_RC_TRANSMITTER_BASE_H

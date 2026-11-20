#ifndef BITSIM_CONTROL_RC_COMMAND_H
#define BITSIM_CONTROL_RC_COMMAND_H

#include "bitsim/bitsim_types.h"

namespace bitsim
{
namespace control
{

///
/// @ingroup bitsim_control
/// @brief What the sticks of a radio transmitter for a quadrotor say, as a toy drone's transmitter
/// sends them: the throttle and three sticks, and whether the motors are armed.
///
/// The sticks follow the usual mode 2 convention: pushing pitch forward flies forward, roll to
/// the right flies to the right, and yaw to the right turns clockwise seen from above.
///
struct RCCommand
{
    real_t throttle{0.0}; ///< [0, 1]: 0 the motors off, 1 full thrust
    real_t yaw{0.0};      ///< [-1, 1]: + turns to the right
    real_t pitch{0.0};    ///< [-1, 1]: + flies forward, nose down
    real_t roll{0.0};     ///< [-1, 1]: + flies to the right, right side down
    bool armed{false};    ///< the motors spin only when armed

    ///
    /// @brief The command with the throttle in [0, 1] and the sticks in [-1, 1]
    ///
    RCCommand clamped() const noexcept;
};

///
/// @ingroup bitsim_control
/// @brief What a receiver does when it loses the transmitter: disarm, so the motors stop
///
inline RCCommand failsafe_command() noexcept { return RCCommand{}; }

inline RCCommand RCCommand::clamped() const noexcept
{
    const auto clamp = [](real_t x, real_t lo, real_t hi)
    { return x < lo ? lo : (x > hi ? hi : x); };
    return {clamp(throttle, 0.0, 1.0), clamp(yaw, -1.0, 1.0), clamp(pitch, -1.0, 1.0),
            clamp(roll, -1.0, 1.0), armed};
}

} // namespace control
} // namespace bitsim

#endif // BITSIM_CONTROL_RC_COMMAND_H

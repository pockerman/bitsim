#ifndef DC_MOTOR_BOARD_BASE_H
#define DC_MOTOR_BOARD_BASE_H

#include "bitrl/bitrl_types.h"

#include <chrono>
#include <optional>

namespace bitrl
{
namespace boards
{

///
/// @brief What a board last reported about one of its motors
///
struct MotorChannelState
{
    ///
    /// @brief The angle of the output shaft [rad], since the board started
    ///
    real_t angle{0.0};

    ///
    /// @brief The speed of the output shaft [rad/s]
    ///
    real_t speed{0.0};

    ///
    /// @brief When the report arrived
    ///
    std::chrono::steady_clock::time_point received_at{};
};

/**
 * @class DCMotorBoardBase
 * @ingroup bitrl_boards
 * @brief A microcontroller board that drives one or more DC motors. The
 * application sets a target speed per motor, and the board runs the motors
 * and reports back what its encoders measure. One board usually drives
 * several motors over a single connection, so the motors share it.
 */
class DCMotorBoardBase
{
  public:
    virtual ~DCMotorBoardBase() = default;

    ///
    /// @brief Connect to the board
    ///
    virtual void connect() = 0;

    ///
    /// @return The number of motors the board drives
    ///
    virtual uint_t n_motors() const noexcept = 0;

    ///
    /// @brief Set the target speed of the output shaft of a motor [rad/s]
    ///
    virtual void set_target_speed(uint_t channel, real_t speed) = 0;

    ///
    /// @brief Exchange messages with the board: send the targets when it is
    /// time and read what has arrived. Cheap, so it is safe to call it often
    ///
    virtual void update() = 0;

    ///
    /// @return The latest report for a motor, or nothing if the board has not
    /// reported yet
    ///
    virtual std::optional<MotorChannelState> motor_state(uint_t channel) const = 0;

    ///
    /// @return True if the board has reported recently
    ///
    virtual bool is_alive() const = 0;

  protected:
    DCMotorBoardBase() = default;
};

} // namespace boards
} // namespace bitrl

#endif // DC_MOTOR_BOARD_BASE_H

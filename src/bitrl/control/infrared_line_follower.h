#ifndef BITRL_CONTROL_INFRARED_LINE_FOLLOWER_H
#define BITRL_CONTROL_INFRARED_LINE_FOLLOWER_H

#include "bitrl/bitrl_types.h"

namespace bitrl
{
namespace sensors
{
class InfraredSensor;
}

namespace control
{

///
/// @ingroup bitrl_control
/// @brief What a differential drive robot is told to do: drive at v [m/s] and turn at omega
/// [rad/s], counterclockwise positive
///
struct DiffDriveCommand
{
    real_t v{0.0};
    real_t omega{0.0};
};

/**
 * @class InfraredLineFollower
 * @ingroup bitrl_control
 * @brief Follow a dark line with a row of infrared elements.
 *
 * The elements, left to right, say where the line is: +1 under the leftmost, 0 under the
 * middle, -1 under the rightmost, the mean over the elements that see it. The robot turns
 * towards the line, harder and slower the further out it is:
 *
 * \f[
 * v = v_{cruise} (1 - |s| / 2), \qquad \omega = \omega_{line}\, s
 * \f]
 *
 * With no element on the line it turns on the spot towards where it saw the line last, and
 * after lost_time it gives up.
 */
class InfraredLineFollower
{
  public:
    struct Parameters
    {
        real_t cruise_speed{0.3};     ///< [m/s] with the line under the middle
        real_t line_turn_rate{1.6};   ///< [rad/s] with only an outer element on the line
        real_t search_turn_rate{0.8}; ///< [rad/s] while looking for the line
        real_t lost_time{3.0};        ///< [s] without the line before giving up
    };

    InfraredLineFollower() = default;
    explicit InfraredLineFollower(const Parameters &parameters) : p_(parameters) {}

    ///
    /// @brief The command for the elements' last reading, dt [s] after the last command
    ///
    DiffDriveCommand command(const sensors::InfraredSensor &ir, real_t dt);

    ///
    /// @brief True when no element saw the line at the last command
    ///
    bool is_lost() const noexcept { return lost_for_ > 0.0; }

    ///
    /// @brief True when no element has seen the line for longer than lost_time
    ///
    bool has_given_up() const noexcept { return lost_for_ > p_.lost_time; }

    const Parameters &parameters() const noexcept { return p_; }

  private:
    Parameters p_{};
    real_t last_side_{0.0};
    real_t lost_for_{0.0};
};

} // namespace control
} // namespace bitrl

#endif // BITRL_CONTROL_INFRARED_LINE_FOLLOWER_H

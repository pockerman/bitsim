#include "bitrl/control/infrared_line_follower.h"
#include "bitrl/sensors/infrared_sensor.h"

#include <cmath>

namespace bitrl
{
namespace control
{

DiffDriveCommand InfraredLineFollower::command(const sensors::InfraredSensor &ir, real_t dt)
{
    // Where the line is: +1 under the left element, 0 under the middle one, -1 under the right
    real_t sum = 0.0;
    uint_t n_dark = 0;
    const uint_t n = ir.n_elements();
    for (uint_t i = 0; i < n; ++i)
    {
        if (ir.is_dark(i))
        {
            sum += n > 1 ? 1.0 - 2.0 * i / (n - 1.0) : 0.0;
            ++n_dark;
        }
    }

    if (n_dark == 0)
    {
        lost_for_ += dt;
        return {0.0, last_side_ >= 0.0 ? p_.search_turn_rate : -p_.search_turn_rate};
    }
    lost_for_ = 0.0;

    const real_t side = sum / n_dark;
    if (side != 0.0)
    {
        last_side_ = side;
    }
    return {p_.cruise_speed * (1.0 - 0.5 * std::abs(side)), p_.line_turn_rate * side};
}

} // namespace control
} // namespace bitrl

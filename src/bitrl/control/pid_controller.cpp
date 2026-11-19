#include "bitrl/control/pid_controller.h"
#include "bitrl/bitrl_types.h"

#include <algorithm>
#include <stdexcept>

namespace bitrl
{
namespace control
{
real_t PIDController::compute_ctrl_signal(real_t current, real_t target, real_t dt)
{
    if (!(dt > 0.0))
    {
        throw std::invalid_argument("PIDController: dt must be positive");
    }

    const real_t err = target - current;
    const real_t derivative = has_err_old_ ? (err - err_old_) / dt : 0.0;
    err_old_ = err;
    has_err_old_ = true;

    const real_t integral = integral_ + err * dt;
    const real_t unlimited = kp_ * err + ki_ * integral + kd_ * derivative;
    const real_t u =
        is_limited() ? std::clamp(unlimited, limits_.first, limits_.second) : unlimited;

    // While the output is at a limit, the integral does not grow
    if (u == unlimited)
    {
        integral_ = integral;
    }
    return u;
}
} // namespace control
} // namespace bitrl

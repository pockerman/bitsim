#include "bitsim/control/pi_controller.h"

#include <algorithm>
#include <stdexcept>

namespace bitsim
{
namespace control
{

PIController::PIController(real_t kp, real_t ki, real_t limit) : kp_(kp), ki_(ki), limit_(limit)
{
    if (limit <= 0.0)
    {
        throw std::invalid_argument("PIController: the limit must be positive");
    }
}

real_t PIController::update(real_t error, real_t dt, real_t feedforward)
{
    const real_t unlimited = feedforward + kp_ * error + ki_ * (integral_ + error * dt);
    const real_t output = std::clamp(unlimited, -limit_, limit_);
    if (output == unlimited)
    {
        integral_ += error * dt;
    }
    return output;
}

} // namespace control
} // namespace bitsim

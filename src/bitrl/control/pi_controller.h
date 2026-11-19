#ifndef BITRL_CONTROL_PI_CONTROLLER_H
#define BITRL_CONTROL_PI_CONTROLLER_H

#include "bitrl/bitrl_types.h"

namespace bitrl
{
namespace control
{

/**
 * @class PIController
 * @ingroup bitrl_control
 * @brief A PI controller whose output is limited to [-limit, limit], with anti-windup and a
 * feedforward term.
 *
 * The output is \f$u = u_{ff} + K_p e + K_i \int e\, dt\f$. While the output is at its limit
 * the integral stops growing; otherwise it would wind up and make the controller overshoot
 * once the error changes sign. The feedforward \f$u_{ff}\f$ is the part of the output that
 * can be predicted without feedback, such as the voltage a motor needs for a speed.
 */
class PIController
{
  public:
    ///
    /// @param kp The proportional gain
    /// @param ki The integral gain
    /// @param limit The largest output, in either direction
    ///
    PIController(real_t kp, real_t ki, real_t limit);

    ///
    /// @brief The output for the error after a step of dt [s]
    ///
    real_t update(real_t error, real_t dt, real_t feedforward = 0.0);

    ///
    /// @brief Forget the integral
    ///
    void reset() { integral_ = 0.0; }

    real_t kp() const noexcept { return kp_; }
    real_t ki() const noexcept { return ki_; }
    real_t limit() const noexcept { return limit_; }
    real_t integral() const noexcept { return integral_; }

  private:
    real_t kp_;
    real_t ki_;
    real_t limit_;
    real_t integral_{0.0};
};

} // namespace control
} // namespace bitrl

#endif // BITRL_CONTROL_PI_CONTROLLER_H

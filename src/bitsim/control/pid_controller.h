#ifndef BITSIM_CONTROL_PID_CONTROLLER_H
#define BITSIM_CONTROL_PID_CONTROLLER_H

#include "bitsim/bitsim_types.h"

#include <stdexcept>
#include <utility>

namespace bitsim
{
namespace control
{
/**
 * @class PIDController
 * @ingroup bitsim_control
 * @brief A PID controller, \f$u = K_p e + K_i \int e\, dt + K_d \dot e\f$ with
 * \f$e\f$ = target - current, and optional limits on its output.
 *
 * The first call after construction or reset() has no derivative term: there is no earlier error
 * to take it from, and taking it from 0 would kick the output. With limits, the integral stops
 * growing while the output is at a limit, so that it does not wind up; see PIController.
 */
class PIDController
{
  public:
    typedef real_t ctrl_signal_type;

    ///
    /// @param Kp The proportional gain
    /// @param Ki The integral gain
    /// @param Kd The derivative gain
    ///
    PIDController(real_t Kp, real_t Ki, real_t Kd);

    real_t get_kp() const noexcept { return kp_; }
    real_t get_ki() const noexcept { return ki_; }
    real_t get_kd() const noexcept { return kd_; }

    /// Test the is_limited_ flag
    /// @return True is the set_control_limits has been called
    bool is_limited() const noexcept { return is_limited_; }

    ///
    /// @brief Limit the output to [limits.first, limits.second]
    ///
    void set_control_limits(std::pair<real_t, real_t> limits);

    ///
    /// @param current The value the controlled system has now
    /// @param target The value it should have
    /// @param dt The time since the last call [s], positive
    /// @return The control signal
    ///
    ctrl_signal_type compute_ctrl_signal(real_t current, real_t target, real_t dt);

    ///
    /// @brief Forget the integral and the last error
    ///
    void reset() noexcept
    {
        integral_ = 0.0;
        has_err_old_ = false;
    }

    real_t integral() const noexcept { return integral_; }

  private:
    bool is_limited_{false};
    real_t kp_;
    real_t ki_;
    real_t kd_;

    real_t integral_{0.0};
    real_t err_old_{0.0};
    bool has_err_old_{false};
    // min = limits.first, max = limits.second
    std::pair<real_t, real_t> limits_;
};

inline PIDController::PIDController(real_t kp, real_t ki, real_t kd) : kp_(kp), ki_(ki), kd_(kd) {}

inline void PIDController::set_control_limits(std::pair<real_t, real_t> limits)
{
    if (!(limits.first < limits.second))
    {
        throw std::invalid_argument("PIDController: the lower limit must be below the upper one");
    }
    is_limited_ = true;
    limits_ = limits;
}
} // namespace control

} // namespace bitsim

#endif // BITSIM_CONTROL_PID_CONTROLLER_H

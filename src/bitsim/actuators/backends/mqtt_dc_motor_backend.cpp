#include "bitsim/bitsim_config.h"

#ifdef BITSIM_MQTT

#include "bitsim/actuators/backends/mqtt_dc_motor_backend.h"

#include <algorithm>
#include <chrono>
#include <stdexcept>

namespace bitsim
{
namespace actuators::backends
{
namespace
{
// Never predict further ahead than this from the last report
const real_t MAX_PREDICTION = 0.1;
} // namespace

const std::string MQTT_DCMotorBackend::BACKEND_TYPE = "MQTT";

MQTT_DCMotorBackend::MQTT_DCMotorBackend(std::shared_ptr<boards::DCMotorBoardBase> board,
                                         uint_t channel, real_t bandwidth)
    : DCMotorBackendBase(BACKEND_TYPE), board_(board), channel_(channel), bandwidth_(bandwidth)
{
    if (!board_)
    {
        throw std::logic_error("MQTT_DCMotorBackend needs a board");
    }
    if (channel_ >= board_->n_motors())
    {
        throw std::logic_error("MQTT_DCMotorBackend: the board has no motor on this channel");
    }
}

void MQTT_DCMotorBackend::set_target_speed(real_t speed)
{
    board_->set_target_speed(channel_, speed);
}

bool MQTT_DCMotorBackend::is_ready() const { return zero_.has_value() && board_->is_alive(); }

real_t MQTT_DCMotorBackend::predicted_angle_() const
{
    return reported_angle_ + reported_speed_ * report_age_;
}

void MQTT_DCMotorBackend::step(real_t dt)
{
    board_->update();

    const auto state = board_->motor_state(channel_);
    if (!state)
    {
        return;
    }

    if (!zero_)
    {
        zero_ = state->angle;
        estimate_ = 0.0;
        estimate_rate_ = state->speed;
    }

    reported_angle_ = state->angle - *zero_;
    reported_speed_ = state->speed;
    report_age_ = std::min(
        std::chrono::duration<real_t>(std::chrono::steady_clock::now() - state->received_at)
            .count(),
        MAX_PREDICTION);

    // If the board goes quiet, the shaft stays where it was
    if (!board_->is_alive())
    {
        estimate_rate_ = 0.0;
        return;
    }

    // A jump of the prediction changes the acceleration of the estimate,
    // never its speed
    const real_t w = bandwidth_;
    const real_t acceleration =
        w * w * (predicted_angle_() - estimate_) + 2.0 * w * (reported_speed_ - estimate_rate_);
    estimate_rate_ += dt * acceleration;
    estimate_ += dt * estimate_rate_;
}

} // namespace actuators::backends
} // namespace bitsim

#endif

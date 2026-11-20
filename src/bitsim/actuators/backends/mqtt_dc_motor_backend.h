#ifndef MQTT_DC_MOTOR_BACKEND_H
#define MQTT_DC_MOTOR_BACKEND_H

#include "bitsim/bitsim_config.h"

#ifdef BITSIM_MQTT

#include "bitsim/actuators/backends/dc_motor_backend_base.h"
#include "bitsim/bitsim_types.h"
#include "bitsim/boards/dc_motor_board_base.h"

#include <memory>
#include <optional>
#include <string>

namespace bitsim
{
namespace actuators::backends
{

/**
 * @class MQTT_DCMotorBackend
 * @ingroup bitsim_actuators
 * @brief A real DC motor on a board. The target speed goes to the board, and
 * the angle comes back from its encoder.
 *
 * The board reports every few tens of milliseconds, while a simulation steps
 * every millisecond or two. In between, the backend predicts the angle from the
 * last report and the speed. A physics engine that follows the angle
 * differentiates it twice, so every correction of the prediction would show up
 * as a spike of torque. So angle() is a smooth estimate instead: a second-order
 * filter pulled towards the predicted angle and the reported speed.
 *
 * Angles are counted from the first report, so the shaft starts at zero.
 */
class MQTT_DCMotorBackend final : public DCMotorBackendBase
{
  public:
    static const std::string BACKEND_TYPE;

    ///
    /// @param board The board the motor is on. Several motors share a board
    /// @param channel The motor on the board
    /// @param bandwidth The bandwidth of the estimate [rad/s]
    ///
    MQTT_DCMotorBackend(std::shared_ptr<boards::DCMotorBoardBase> board, uint_t channel,
                        real_t bandwidth = 30.0);

    OutputType output_type() const noexcept override { return OutputType::ANGLE; }

    void set_target_speed(real_t speed) override;

    void step(real_t dt) override;

    void step(real_t dt, const ShaftState &) override { step(dt); }

    real_t angle() const override { return estimate_; }

    real_t speed() const override { return estimate_rate_; }

    ///
    /// @return What the board's encoder last reported: reported_angle(). Unlike
    /// angle(), it is not smoothed or predicted
    ///
    real_t encoder_angle() const override { return reported_angle_; }

    ///
    /// @return True once the board has reported, and while it keeps reporting
    ///
    bool is_ready() const override;

    ///
    /// @return The angle the board last reported, from the first report [rad]
    ///
    real_t reported_angle() const noexcept { return reported_angle_; }

    ///
    /// @return The speed the board last reported [rad/s]
    ///
    real_t reported_speed() const noexcept { return reported_speed_; }

    std::shared_ptr<boards::DCMotorBoardBase> board() const noexcept { return board_; }
    uint_t channel() const noexcept { return channel_; }

  private:
    real_t predicted_angle_() const;

    std::shared_ptr<boards::DCMotorBoardBase> board_;
    uint_t channel_;
    real_t bandwidth_;

    std::optional<real_t> zero_;
    real_t reported_angle_{0.0};
    real_t reported_speed_{0.0};
    real_t report_age_{0.0};

    real_t estimate_{0.0};
    real_t estimate_rate_{0.0};
};

} // namespace actuators::backends
} // namespace bitsim

#endif
#endif // MQTT_DC_MOTOR_BACKEND_H

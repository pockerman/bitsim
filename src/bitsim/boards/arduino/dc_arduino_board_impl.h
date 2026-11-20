#ifndef DC_ARDUINO_BOARD_IMPL_H
#define DC_ARDUINO_BOARD_IMPL_H

#include "bitsim/bitsim_config.h"

#ifdef BITSIM_MQTT

#include "bitsim/bitsim_types.h"
#include "bitsim/boards/dc_motor_board_base.h"
#include "bitsim/network/mqtt_publisher.h"
#include "bitsim/network/mqtt_subscriber.h"

#include <array>
#include <chrono>
#include <optional>
#include <string>
#include <vector>

namespace bitsim
{
namespace boards::arduino
{

/**
 * @class DCArduinoBoardImpl
 * @ingroup bitsim_boards
 * @brief An Arduino that drives DC motors with encoders and talks over MQTT.
 *
 * The application publishes the target speeds on the command topic, and the
 * board publishes the distance each wheel has covered on the state topic:
 *
 * @code
 * command: {"seq": 12, "left_speed": 2.5, "right_speed": 2.5}
 * state:   {"ack": 12, "left_distance": 0.53, "right_distance": 0.52,
 *           "left_speed": 2.49, "right_speed": 2.51, "t_ms": 81234}
 * @endcode
 *
 * Speeds are wheel speeds in rad/s and distances are in m. The board turns
 * encoder counts into distance with its wheel radius, so wheel_radius here
 * must match the one in the sketch. `ack` is the number of the last command
 * the board received, which gives the round trip of the messages.
 */
class DCArduinoBoardImpl final : public DCMotorBoardBase
{
  public:
    using Clock = std::chrono::steady_clock;

    struct Config
    {
        std::string broker{"tcp://localhost:1883"};
        std::string command_topic{"bitsim/dc_board/command"};
        std::string state_topic{"bitsim/dc_board/state"};

        ///
        /// @brief The names of the motors, in channel order. They prefix the
        /// keys of the messages
        ///
        std::vector<std::string> channels{"left", "right"};

        ///
        /// @brief The radius the board uses to turn angles into distances [m]
        ///
        real_t wheel_radius{0.1};

        ///
        /// @brief How often the targets are sent [s]
        ///
        real_t command_period{0.02};

        ///
        /// @brief No report for this long means the board is gone [s]
        ///
        real_t timeout{0.5};
    };

    explicit DCArduinoBoardImpl(const Config &config);

    void connect() override;

    uint_t n_motors() const noexcept override { return config_.channels.size(); }

    void set_target_speed(uint_t channel, real_t speed) override;

    void update() override;

    std::optional<MotorChannelState> motor_state(uint_t channel) const override;

    bool is_alive() const override;

    ///
    /// @brief Wait until the board reports, for at most timeout
    /// @return True if it did
    ///
    bool wait_for_board(std::chrono::milliseconds timeout);

    const Config &config() const noexcept { return config_; }

    ///
    /// @return The last round trip: from sending a command until hearing that
    /// the board has it [s]
    ///
    real_t round_trip() const noexcept { return round_trip_; }
    real_t mean_round_trip() const noexcept
    {
        return n_round_trips_ ? round_trip_sum_ / n_round_trips_ : 0.0;
    }
    real_t max_round_trip() const noexcept { return round_trip_max_; }
    uint_t n_sent() const noexcept { return n_sent_; }
    uint_t n_received() const noexcept { return n_received_; }

  private:
    void send_command_(Clock::time_point now);
    void receive_(const std::string &text);

    Config config_;
    network::MqttSubscriber state_;
    network::MqttPublisher commands_;

    std::vector<real_t> targets_;
    std::vector<std::optional<MotorChannelState>> states_;
    std::optional<Clock::time_point> last_received_;
    std::optional<Clock::time_point> last_sent_;

    long seq_{0};
    std::array<Clock::time_point, 256> sent_at_{};
    long last_ack_{0};
    uint_t n_sent_{0};
    uint_t n_received_{0};

    real_t round_trip_{0.0};
    real_t round_trip_sum_{0.0};
    real_t round_trip_max_{0.0};
    uint_t n_round_trips_{0};
};

} // namespace boards::arduino
} // namespace bitsim

#endif
#endif // DC_ARDUINO_BOARD_IMPL_H

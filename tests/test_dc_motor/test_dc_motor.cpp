#include "bitsim/actuators/backends/sim_dc_motor_backend.h"
#include "bitsim/actuators/dc_motor.h"
#include "bitsim/bitsim_config.h"
#include "bitsim/bitsim_consts.h"
#include "bitsim/bitsim_types.h"

#ifdef BITSIM_MQTT
#include "bitsim/actuators/backends/mqtt_dc_motor_backend.h"
#include "bitsim/boards/dc_motor_board_base.h"
#endif

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>

namespace
{

using bitsim::real_t;
using bitsim::uint_t;
using bitsim::actuators::DCMotor;
using bitsim::actuators::ShaftState;
using bitsim::actuators::backends::DCMotorParameters;
using bitsim::actuators::backends::SIM_DCMotorBackend;

void run_free(SIM_DCMotorBackend &motor, real_t duration, real_t dt = 0.001)
{
    for (int i = 0; i < static_cast<int>(std::round(duration / dt)); ++i)
    {
        motor.step(dt);
    }
}

} // namespace

TEST(TestSimDCMotorBackend, ReachesTheTargetSpeed)
{
    SIM_DCMotorBackend motor;
    motor.set_target_speed(2.5);
    run_free(motor, 2.0);

    EXPECT_NEAR(motor.speed(), 2.5, 0.1);
    EXPECT_NEAR(motor.measured_speed(), 2.5, 0.2);
    EXPECT_GT(motor.angle(), 2.5);
}

TEST(TestSimDCMotorBackend, RunsBackwards)
{
    SIM_DCMotorBackend motor;
    motor.set_target_speed(-4.0);
    run_free(motor, 2.0);

    EXPECT_NEAR(motor.speed(), -4.0, 0.15);
    EXPECT_LT(motor.angle(), 0.0);
}

TEST(TestSimDCMotorBackend, EncoderCountsWholeSteps)
{
    SIM_DCMotorBackend motor;
    motor.set_target_speed(1.0);
    run_free(motor, 1.0);

    const real_t resolution = 2.0 * bitsim::consts::maths::PI / motor.parameters().encoder_counts;
    EXPECT_LE(motor.encoder_angle(), motor.angle());
    EXPECT_GT(motor.encoder_angle(), motor.angle() - resolution);
    const real_t counts = motor.encoder_angle() / resolution;
    EXPECT_NEAR(counts, std::round(counts), 1.0e-6);
}

TEST(TestSimDCMotorBackend, GivesTheStallTorqueOnAHeldShaft)
{
    // A target far above the no-load speed saturates the controller, so the
    // full supply voltage is across the windings of a shaft that cannot turn
    SIM_DCMotorBackend motor;
    motor.set_target_speed(100.0);
    for (int i = 0; i < 100; ++i)
    {
        motor.step(0.001, ShaftState{0.0, 0.0});
    }

    EXPECT_NEAR(motor.voltage(), motor.parameters().supply_voltage, 1.0e-9);
    EXPECT_NEAR(motor.torque(), motor.parameters().stall_torque(), 1.0e-3);
    EXPECT_DOUBLE_EQ(motor.angle(), 0.0);
}

TEST(TestSimDCMotorBackend, LoadSlowsTheMotor)
{
    SIM_DCMotorBackend free_motor;
    SIM_DCMotorBackend loaded_motor;
    loaded_motor.set_load_torque(0.3);

    // Open loop, the controller cannot tell them apart
    DCMotorParameters parameters;
    free_motor.set_target_speed(parameters.no_load_speed());
    loaded_motor.set_target_speed(parameters.no_load_speed());
    run_free(free_motor, 1.0);
    run_free(loaded_motor, 1.0);

    EXPECT_LT(loaded_motor.speed(), free_motor.speed() - 1.0);
}

TEST(TestDCMotorParameters, ReadsOnlyWhatIsGiven)
{
    DCMotorParameters parameters;
    parameters.read(nlohmann::json{{"gear_ratio", 50.0}, {"encoder_counts", 1200}});

    EXPECT_DOUBLE_EQ(parameters.gear_ratio, 50.0);
    EXPECT_EQ(parameters.encoder_counts, 1200u);
    EXPECT_DOUBLE_EQ(parameters.resistance, DCMotorParameters().resistance);
}

TEST(TestSimDCMotorBackend, RejectsBadParameters)
{
    DCMotorParameters parameters;
    parameters.resistance = 0.0;
    EXPECT_THROW(SIM_DCMotorBackend{parameters}, std::logic_error);
}

TEST(TestDCMotor, ClampsTheTargetToTheMaximumSpeed)
{
    auto motor = DCMotor::create_simulated(DCMotorParameters(), "left");
    motor->set_max_speed(3.0);

    motor->set_speed(10.0);
    EXPECT_DOUBLE_EQ(motor->target_speed(), 3.0);
    motor->set_speed(-10.0);
    EXPECT_DOUBLE_EQ(motor->target_speed(), -3.0);

    EXPECT_EQ(motor->name(), "left");
    EXPECT_EQ(motor->backend_type_str(), SIM_DCMotorBackend::BACKEND_TYPE);
    EXPECT_EQ(motor->output_type(), DCMotor::OutputType::TORQUE);
}

TEST(TestDCMotor, NeedsABackend) { EXPECT_THROW(DCMotor(nullptr), std::logic_error); }

#ifdef BITSIM_MQTT
namespace
{

// A board in memory: the test decides what it reports
class FakeBoard : public bitsim::boards::DCMotorBoardBase
{
  public:
    void connect() override {}
    uint_t n_motors() const noexcept override { return 2; }
    void set_target_speed(uint_t channel, real_t speed) override { targets.at(channel) = speed; }
    void update() override { ++n_updates; }
    std::optional<bitsim::boards::MotorChannelState> motor_state(uint_t channel) const override
    {
        return states.at(channel);
    }
    bool is_alive() const override { return alive; }

    void report(uint_t channel, real_t angle, real_t speed)
    {
        bitsim::boards::MotorChannelState state;
        state.angle = angle;
        state.speed = speed;
        state.received_at = std::chrono::steady_clock::now();
        states.at(channel) = state;
        alive = true;
    }

    std::vector<real_t> targets{0.0, 0.0};
    std::vector<std::optional<bitsim::boards::MotorChannelState>> states{2};
    bool alive = false;
    int n_updates = 0;
};

using bitsim::actuators::backends::MQTT_DCMotorBackend;

} // namespace

TEST(TestMqttDCMotorBackend, SendsTheTargetToItsChannel)
{
    auto board = std::make_shared<FakeBoard>();
    DCMotor motor(std::make_shared<MQTT_DCMotorBackend>(board, 1));
    motor.set_speed(2.5);

    EXPECT_DOUBLE_EQ(board->targets[0], 0.0);
    EXPECT_DOUBLE_EQ(board->targets[1], 2.5);
    EXPECT_EQ(motor.output_type(), DCMotor::OutputType::ANGLE);
}

TEST(TestMqttDCMotorBackend, IsNotReadyBeforeTheBoardReports)
{
    auto board = std::make_shared<FakeBoard>();
    MQTT_DCMotorBackend motor(board, 0);
    motor.step(0.01);

    EXPECT_FALSE(motor.is_ready());
    EXPECT_EQ(board->n_updates, 1);
}

TEST(TestMqttDCMotorBackend, CountsFromTheFirstReportAndFollowsIt)
{
    auto board = std::make_shared<FakeBoard>();
    MQTT_DCMotorBackend motor(board, 0);

    // The board has been running for a while
    board->report(0, 7.0, 0.0);
    motor.step(0.01);
    EXPECT_TRUE(motor.is_ready());
    EXPECT_DOUBLE_EQ(motor.angle(), 0.0);

    // It turns by one radian and stops. The estimate settles there smoothly
    board->report(0, 8.0, 0.0);
    real_t largest_rate = 0.0;
    for (int i = 0; i < 300; ++i)
    {
        motor.step(0.01);
        largest_rate = std::max(largest_rate, std::abs(motor.speed()));
    }
    EXPECT_NEAR(motor.reported_angle(), 1.0, 1.0e-12);
    EXPECT_NEAR(motor.angle(), 1.0, 1.0e-3);
    EXPECT_LT(largest_rate, 30.0 / 2.0);
}

TEST(TestMqttDCMotorBackend, HoldsWhenTheBoardGoesQuiet)
{
    auto board = std::make_shared<FakeBoard>();
    MQTT_DCMotorBackend motor(board, 0);
    board->report(0, 0.0, 0.0);
    motor.step(0.01);

    board->report(0, 0.5, 1.0);
    for (int i = 0; i < 20; ++i)
    {
        motor.step(0.01);
    }
    board->alive = false;
    const real_t held = motor.angle();
    for (int i = 0; i < 20; ++i)
    {
        motor.step(0.01);
    }

    EXPECT_FALSE(motor.is_ready());
    EXPECT_DOUBLE_EQ(motor.angle(), held);
    EXPECT_DOUBLE_EQ(motor.speed(), 0.0);
}

TEST(TestMqttDCMotorBackend, RejectsAMissingChannel)
{
    auto board = std::make_shared<FakeBoard>();
    EXPECT_THROW(MQTT_DCMotorBackend(board, 2), std::logic_error);
    EXPECT_THROW(MQTT_DCMotorBackend(nullptr, 0), std::logic_error);
}
#endif

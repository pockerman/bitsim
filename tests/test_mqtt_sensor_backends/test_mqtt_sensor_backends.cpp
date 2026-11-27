#include "bitsim/network/mqtt_publisher.h"
#include "bitsim/sensors/backends/mqtt_imu_backend.h"
#include "bitsim/sensors/backends/mqtt_infrared_backend.h"
#include "bitsim/sensors/backends/mqtt_ultrasound_backend.h"
#include "bitsim/sensors/backends/range_sensor_backend_base.h"
#include "bitsim/sensors/messages/ultrasound.h"

#include <gtest/gtest.h>

#include <chrono>
#include <stdexcept>
#include <string>
#include <thread>

namespace
{
using bitsim::real_t;
using bitsim::sensors::UltrasoundMessage;
using bitsim::sensors::backends::MQTT_InfraredBackend;
using bitsim::sensors::backends::MQTT_SensorChannel;
using bitsim::sensors::backends::MQTT_UltrasonicBackend;

const std::string BROKER = "tcp://localhost:1883";
} // namespace

TEST(TestMqttSensorBackends, UltrasoundMessagesAreInMetres)
{
    const auto cm = UltrasoundMessage::parse(R"({"distance": 62.0, "unit": "cm"})");
    ASSERT_TRUE(cm);
    EXPECT_DOUBLE_EQ(cm->metres(), 0.62);

    const auto m = UltrasoundMessage::parse(R"({"distance": 1.5})");
    ASSERT_TRUE(m);
    EXPECT_DOUBLE_EQ(m->metres(), 1.5);

    EXPECT_TRUE(UltrasoundMessage::parse(
        R"({"distance": 120, "unit": "mm", "timestamp": "2026-11-18 10:15:02"})"));
    EXPECT_FALSE(UltrasoundMessage::parse("{"));
    EXPECT_FALSE(UltrasoundMessage::parse(R"({"unit": "m"})"));
    EXPECT_FALSE(UltrasoundMessage::parse(R"({"distance": 1.0, "unit": "feet"})"));
    EXPECT_FALSE(UltrasoundMessage::parse(R"({"distance": "far"})"));
}

TEST(TestMqttSensorBackends, UltrasonicBackendKeepsTheLatestValidReading)
{
    MQTT_UltrasonicBackend backend;
    EXPECT_FALSE(backend.is_alive());
    EXPECT_TRUE(backend.receive(R"({"distance": 80, "unit": "cm"})"));
    EXPECT_FALSE(backend.receive("garbage"));
    EXPECT_TRUE(backend.is_alive());
    EXPECT_DOUBLE_EQ(backend.last_value_read().metres(), 0.8);

    // Reading needs a connection: a forgotten connect() must not look like a
    // clear path
    EXPECT_THROW(backend.read_values(), std::logic_error);
}

TEST(TestMqttSensorBackends, LoadsTheChannelAndRangeFromJson)
{
    MQTT_UltrasonicBackend backend;
    backend.load(nlohmann::json::parse(R"({"max_distance": 4.0, "min_distance": 0.02,
        "broker": "tcp://robot:1883", "topic": "odysseus/sonar", "timeout": 0.3})"));
    EXPECT_DOUBLE_EQ(backend.max_distance(), 4.0);
    EXPECT_EQ(backend.channel().config().broker, "tcp://robot:1883");
    EXPECT_EQ(backend.channel().config().topic, "odysseus/sonar");
    EXPECT_DOUBLE_EQ(backend.channel().config().timeout, 0.3);

    EXPECT_EQ(bitsim::sensors::backends::RangeSensorBackendBase::create("MQTT")->backend_type_str(),
              "MQTT");
    EXPECT_THROW(MQTT_UltrasonicBackend().connect(), std::logic_error); // no topic
}

TEST(TestMqttSensorBackends, InfraredBackendNeedsOneValuePerElement)
{
    MQTT_InfraredBackend backend;
    backend.set_elements({{0.2, 0.04, 0.0}, {0.2, 0.0, 0.0}, {0.2, -0.04, 0.0}});
    EXPECT_FALSE(backend.receive(R"({"values": [0.9, 0.1]})"));
    EXPECT_FALSE(backend.is_alive());
    EXPECT_TRUE(backend.receive(R"({"values": [0.9, 0.1, 0.8]})"));
    EXPECT_TRUE(backend.is_alive());
    EXPECT_DOUBLE_EQ(backend.last_value_read().values[1], 0.1);
    EXPECT_EQ(backend.channel().n_received(), 1u);
}

TEST(TestMqttSensorBackends, IMUBackendKeepsTheLatestValidReading)
{
    bitsim::sensors::backends::MQTT_IMUBackend backend;
    EXPECT_FALSE(backend.receive(R"({"accel": [0, 0, 9.8]})"));
    EXPECT_FALSE(backend.is_alive());
    EXPECT_TRUE(backend.receive(R"({"accel": [0.1, 0, 9.8], "gyro": [0, 0, 0.5]})"));
    EXPECT_TRUE(backend.is_alive());
    EXPECT_DOUBLE_EQ(backend.last_value_read().gyro[2], 0.5);
    EXPECT_THROW(backend.read_values(), std::logic_error); // not connected
}

// Talks to a broker on this computer, if there is one
TEST(TestMqttSensorBackends, ReadsTheLatestMessagesFromTheBrokerWithoutWaiting)
{
    MQTT_SensorChannel::Config sonar_config;
    sonar_config.broker = BROKER;
    sonar_config.topic = "bitsim/test/sonar";
    MQTT_UltrasonicBackend sonar(sonar_config);
    sonar.set_max_distance(4.0);

    MQTT_SensorChannel::Config ir_config = sonar_config;
    ir_config.topic = "bitsim/test/ir";
    MQTT_InfraredBackend ir(ir_config);
    ir.set_elements({{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}});

    bitsim::network::MqttPublisher sonar_board(BROKER, sonar_config.topic);
    bitsim::network::MqttPublisher ir_board(BROKER, ir_config.topic);
    try
    {
        sonar.connect();
        ir.connect();
        sonar_board.connect();
        ir_board.connect();
    }
    catch (const std::exception &e)
    {
        GTEST_SKIP() << "No MQTT broker at " << BROKER << ": " << e.what();
    }

    // Nothing yet: the path looks clear and the floor white, but not alive
    EXPECT_DOUBLE_EQ(sonar.read_values().at(0), 4.0);
    EXPECT_EQ(ir.read_values(), (std::vector<real_t>{1.0, 1.0}));
    EXPECT_FALSE(sonar.is_alive());

    for (int i = 1; i <= 5; ++i)
    {
        sonar_board.publish(R"({"distance": )" + std::to_string(10 * i) + R"(, "unit": "cm"})", 1);
    }
    ir_board.publish(std::string(R"({"values": [0.9, 0.2]})"), 1);

    // Wait for the messages to arrive, then one read takes all of them
    const auto start = std::chrono::steady_clock::now();
    while (sonar.channel().n_received() < 5 &&
           std::chrono::steady_clock::now() - start < std::chrono::seconds(3))
    {
        sonar.read_values();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    EXPECT_NEAR(sonar.read_values().at(0), 0.5, 1.0e-12);
    EXPECT_TRUE(sonar.is_alive());

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    EXPECT_EQ(ir.read_values(), (std::vector<real_t>{0.9, 0.2}));

    // read_values returns at once when nothing new has arrived
    const auto before = std::chrono::steady_clock::now();
    sonar.read_values();
    EXPECT_LT(std::chrono::steady_clock::now() - before, std::chrono::milliseconds(50));
}

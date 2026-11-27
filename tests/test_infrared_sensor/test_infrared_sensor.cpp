#include "bitsim/bitsim_config.h"
#include "bitsim/bitsim_consts.h"
#include "bitsim/bitsim_types.h"
#include "bitsim/rigid_bodies/chrono_robots/diff_drive_robot.h"
#include "bitsim/sensors/backends/chrono_infrared_backend.h"
#include "bitsim/sensors/backends/infrared_backend_base.h"
#include "bitsim/sensors/infrared_sensor.h"
#include "bitsim/sensors/messages/infrared.h"

#ifdef BITSIM_OPENCV
#include "bitsim/sensors/downward_camera.h"
#include <opencv2/core.hpp>
#endif


#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <memory>
#include <stdexcept>

namespace
{
using bitsim::real_t;
using bitsim::rb::bitsim_chrono::CHRONO_DiffDriveRobot;
using bitsim::sensors::InfraredMessage;
using bitsim::sensors::InfraredSensor;
using bitsim::sensors::backends::CHRONO_InfraredBackend;
using bitsim::sensors::backends::InfraredBackendBase;

// A floor that is white, with a dark line 0.05 m wide along the x axis
real_t striped_floor(real_t, real_t y) { return std::abs(y) < 0.025 ? 0.1 : 0.9; }

std::shared_ptr<chrono::ChBody> make_body(const chrono::ChVector3d &pos, real_t yaw)
{
    auto body = chrono_types::make_shared<chrono::ChBody>();
    body->SetPos(pos);
    body->SetRot(chrono::QuatFromAngleZ(yaw));
    return body;
}

} // namespace

TEST(TestInfraredSensor, EachElementReadsTheFloorBelowIt)
{
    // The body faces along y, so its left is -x
    auto body = make_body(chrono::ChVector3d(0.0, 0.0, 0.2), bitsim::consts::maths::PI / 2.0);
    CHRONO_InfraredBackend backend(body, [](real_t x, real_t y) { return 0.5 + x + 0.1 * y; });
    backend.set_elements({{0.1, 0.2, -0.1}, {0.1, 0.0, -0.1}});

    const auto values = backend.read_values();
    ASSERT_EQ(values.size(), 2u);
    EXPECT_NEAR(values[0], 0.5 - 0.2 + 0.01, 1.0e-12); // at (-0.2, 0.1)
    EXPECT_NEAR(values[1], 0.5 + 0.01, 1.0e-12);       // at (0, 0.1)
}

TEST(TestInfraredSensor, ValuesStayBetweenBlackAndWhite)
{
    CHRONO_InfraredBackend backend(make_body({0.0, 0.0, 0.0}, 0.0),
                                   [](real_t x, real_t) { return x; });
    backend.set_elements({{-2.0, 0.0, 0.0}, {2.0, 0.0, 0.0}});
    const auto values = backend.read_values();
    EXPECT_DOUBLE_EQ(values[0], 0.0);
    EXPECT_DOUBLE_EQ(values[1], 1.0);
}

TEST(TestInfraredSensor, NeedsABodyAndAFloor)
{
    CHRONO_InfraredBackend backend;
    EXPECT_THROW(backend.read_values(), std::logic_error);
    backend.set_chrono_body(make_body({0.0, 0.0, 0.0}, 0.0));
    EXPECT_THROW(backend.read_values(), std::logic_error);
    backend.set_floor(striped_floor);
    EXPECT_NO_THROW(backend.read_values());
}

TEST(TestInfraredSensor, TellsWhichElementsAreOverTheLine)
{
    auto backend = std::make_shared<CHRONO_InfraredBackend>(make_body({0.0, 0.01, 0.0}, 0.0),
                                                            striped_floor);
    backend->set_elements({{0.0, 0.04, 0.0}, {0.0, 0.0, 0.0}, {0.0, -0.04, 0.0}});

    InfraredSensor sensor(backend, "line");
    sensor.init();
    EXPECT_EQ(sensor.n_elements(), 3u);
    EXPECT_FALSE(sensor.is_dark(1)); // all white until the first read

    sensor.read_values();
    EXPECT_FALSE(sensor.is_dark(0)); // y = 0.05
    EXPECT_TRUE(sensor.is_dark(1));  // y = 0.01
    EXPECT_FALSE(sensor.is_dark(2)); // y = -0.03
}

TEST(TestInfraredSensor, ThresholdMustBeBetweenBlackAndWhite)
{
    InfraredSensor sensor(std::make_shared<CHRONO_InfraredBackend>(), "line");
    EXPECT_THROW(sensor.set_threshold(0.0), std::logic_error);
    EXPECT_THROW(sensor.set_threshold(1.0), std::logic_error);
    sensor.set_threshold(0.3);
    EXPECT_DOUBLE_EQ(sensor.threshold(), 0.3);
}

TEST(TestInfraredSensor, CreatesBackendsByTheirRobotFileNames)
{
    EXPECT_EQ(InfraredBackendBase::create("SIMULATION")->backend_type_str(), "SIMULATION");
    EXPECT_THROW(InfraredBackendBase::create("NO_SUCH_BACKEND"), std::runtime_error);
    EXPECT_THROW(CHRONO_InfraredBackend().set_elements({}), std::logic_error);
}

TEST(TestInfraredSensor, LoadsElementsAndRateFromJson)
{
    CHRONO_InfraredBackend backend;
    backend.load(nlohmann::json::parse(R"({"elements": [[0.1, 0.02, 0], [0.1, -0.02, 0]],
                                           "update_rate": 100.0})"));
    EXPECT_EQ(backend.n_elements(), 2u);
    EXPECT_EQ(backend.n_read_values(), 2u);
    EXPECT_DOUBLE_EQ(backend.elements()[1][1], -0.02);
    EXPECT_DOUBLE_EQ(backend.sampling_period(), 0.01);
}

TEST(TestInfraredSensor, ParsesMessages)
{
    const auto message = InfraredMessage::parse(R"({"values": [0.9, 1.4, -0.2]})");
    ASSERT_TRUE(message);
    ASSERT_EQ(message->values.size(), 3u);
    EXPECT_DOUBLE_EQ(message->values[0], 0.9);
    EXPECT_DOUBLE_EQ(message->values[1], 1.0);
    EXPECT_DOUBLE_EQ(message->values[2], 0.0);

    EXPECT_TRUE(InfraredMessage::parse(R"({"values": [0.5], "timestamp": "2026-11-18 10:15:02"})"));
    EXPECT_FALSE(InfraredMessage::parse("not json"));
    EXPECT_FALSE(InfraredMessage::parse(R"({"values": []})"));
    EXPECT_FALSE(InfraredMessage::parse(R"({"values": [0.5, "dark"]})"));
    EXPECT_FALSE(InfraredMessage::parse(R"({"values": [0.5], "timestamp": "yesterday"})"));
}

TEST(TestInfraredSensor, TheLineSensorOfTheRobotFileFollowsTheRobot)
{
    CHRONO_DiffDriveRobot robot;
    robot.load(std::filesystem::path(bitsim::consts::ROBOTS_DIR + "/bitsim_diff_drive_robot.json"));

    auto line = robot.get_line_sensor();
    ASSERT_TRUE(line);
    EXPECT_TRUE(line->is_enabled());
    EXPECT_EQ(line->n_elements(), 3u);
    EXPECT_EQ(line->backend_type_str(), "SIMULATION");

    // Without a floor the elements see nothing
    EXPECT_THROW(line->read_values(), std::logic_error);

    // A line along y at x = 0.5
    robot.set_floor([](real_t x, real_t) { return std::abs(x - 0.5) < 0.025 ? 0.1 : 0.9; });

    // Facing along y with the axle centre at x = 0.5, the line runs under the
    // middle element. The elements are 0.34 m ahead of the axle
    robot.set_world_pose(0.5, 0.0, bitsim::consts::maths::PI / 2.0);
    line->read_values();
    EXPECT_FALSE(line->is_dark(0));
    EXPECT_TRUE(line->is_dark(1));
    EXPECT_FALSE(line->is_dark(2));

    // Moved 4 cm to the right of the line, the left element is over it
    robot.set_world_pose(0.54, 0.0, bitsim::consts::maths::PI / 2.0);
    line->read_values();
    EXPECT_TRUE(line->is_dark(0));
    EXPECT_FALSE(line->is_dark(1));
    EXPECT_FALSE(line->is_dark(2));
}

#ifdef BITSIM_OPENCV
TEST(TestInfraredSensor, AFloorImageIsAsBrightAsItsPixels)
{
    // 2 m x 1 m, 1 cm a pixel, white with a black pixel at the origin
    cv::Mat image(100, 200, CV_8UC3, cv::Scalar(255, 255, 255));
    image.at<cv::Vec3b>(50, 100) = cv::Vec3b(0, 0, 0);
    image.at<cv::Vec3b>(10, 10) = cv::Vec3b(128, 128, 128);
    const bitsim::sensors::FloorImage floor(image, 2.0);

    EXPECT_DOUBLE_EQ(floor.reflectance(0.003, -0.003), 0.0);
    EXPECT_DOUBLE_EQ(floor.reflectance(0.3, 0.2), 1.0);
    EXPECT_NEAR(floor.reflectance(-0.895, 0.395), 128.0 / 255.0, 1.0e-9);
    EXPECT_DOUBLE_EQ(floor.reflectance(5.0, 0.0), 1.0); // outside is white
}
#endif

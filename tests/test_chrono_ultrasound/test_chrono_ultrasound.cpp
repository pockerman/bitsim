#include "bitrl/bitrl_config.h"
#include "bitrl/bitrl_types.h"
#include "bitrl/bitrl_consts.h"
#include "bitrl/rigid_bodies/chrono_robots/diff_drive_robot.h"
#include "bitrl/sensors/backends/chrono_ultrasound_backend.h"
#include "bitrl/sensors/backends/range_sensor_backend_base.h"
#include "bitrl/sensors/ultrasonic_sensor.h"

#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChSystemNSC.h>

#include <cmath>

#include <gtest/gtest.h>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>

namespace
{

using bitrl::real_t;
using bitrl::sensors::backends::CHRONO_UltrasonicBackend;

const real_t MAX_DISTANCE = 5.0;

std::shared_ptr<chrono::ChBody> add_fixed_box(chrono::ChSystemNSC &system,
                                              const chrono::ChVector3d &position, bool collide,
                                              real_t size = 1.0)
{
    auto material = chrono_types::make_shared<chrono::ChContactMaterialNSC>();
    auto box = chrono_types::make_shared<chrono::ChBodyEasyBox>(size, size, size, 1000.0, false,
                                                                collide, material);
    box->SetPos(position);
    box->SetFixed(true);
    system.Add(box);
    return box;
}

real_t read_distance(chrono::ChSystemNSC &system, std::shared_ptr<chrono::ChBody> sensor_body)
{
    CHRONO_UltrasonicBackend backend(system, sensor_body);
    backend.set_max_distance(MAX_DISTANCE);

    // the collision system is populated by the first step
    system.DoStepDynamics(0.001);
    return backend.read_values().at(0);
}

} // namespace

TEST(TestChronoUltrasound, ReadsDistanceToObstacle)
{
    chrono::ChSystemNSC system;
    system.SetCollisionSystemType(chrono::ChCollisionSystem::Type::BULLET);

    auto sensor_body = add_fixed_box(system, chrono::ChVector3d(0.0, 0.0, 0.0), false);

    // the face of the obstacle looking at the sensor is at x = 1.5
    add_fixed_box(system, chrono::ChVector3d(2.0, 0.0, 0.0), true);

    EXPECT_NEAR(read_distance(system, sensor_body), 1.5, 1.0e-2);
}

TEST(TestChronoUltrasound, ReturnsMaxDistanceWhenNothingIsInRange)
{
    chrono::ChSystemNSC system;
    system.SetCollisionSystemType(chrono::ChCollisionSystem::Type::BULLET);

    auto sensor_body = add_fixed_box(system, chrono::ChVector3d(0.0, 0.0, 0.0), false);

    // behind the sensor
    add_fixed_box(system, chrono::ChVector3d(-2.0, 0.0, 0.0), true);

    EXPECT_DOUBLE_EQ(read_distance(system, sensor_body), MAX_DISTANCE);
}

TEST(TestChronoUltrasound, LoadsBothDistanceLimitsFromJson)
{
    CHRONO_UltrasonicBackend backend;
    backend.load_from_json(bitrl::consts::SENSORS_DIR + "/ultrasonic_chrono_backend.json");

    EXPECT_DOUBLE_EQ(backend.max_distance(), 5.0);
    EXPECT_DOUBLE_EQ(backend.min_distance(), 0.5);
}

TEST(TestChronoUltrasound, DisabledSensorDoesNotReadTheBackend)
{
    // no system or body is attached so reading from the backend would fail
    auto backend = std::make_shared<CHRONO_UltrasonicBackend>();

    bitrl::sensors::UltrasonicSensor sensor(backend, "disabled");
    ASSERT_FALSE(sensor.is_enabled());
    EXPECT_TRUE(sensor.read_values().empty());
}

TEST(TestChronoUltrasound, LoadsTheUpdateRateAsASamplingPeriod)
{
    CHRONO_UltrasonicBackend backend;
    backend.load_from_json(bitrl::consts::SENSORS_DIR + "/ultrasonic_chrono_backend.json");

    // 50 readings a second
    EXPECT_DOUBLE_EQ(backend.sampling_period(), 0.02);
    EXPECT_DOUBLE_EQ(backend.position().x(), 0.3);
    EXPECT_DOUBLE_EQ(backend.position().z(), 0.1);
}

TEST(TestChronoUltrasound, CreatesBackendsByTheirRobotFileNames)
{
    using bitrl::sensors::backends::RangeSensorBackendBase;
    EXPECT_EQ(RangeSensorBackendBase::create("SIMULATION")->backend_type_str(), "SIMULATION");
    EXPECT_EQ(RangeSensorBackendBase::create("CHRONO")->backend_type_str(), "SIMULATION");
    EXPECT_THROW(RangeSensorBackendBase::create("NO_SUCH_BACKEND"), std::runtime_error);
}

TEST(TestChronoUltrasound, ReportsMinDistanceForCloserObstacles)
{
    chrono::ChSystemNSC system;
    system.SetCollisionSystemType(chrono::ChCollisionSystem::Type::BULLET);

    auto sensor_body = add_fixed_box(system, chrono::ChVector3d(0.0, 0.0, 0.0), false);
    add_fixed_box(system, chrono::ChVector3d(0.6, 0.0, 0.0), true); // face at x = 0.1

    CHRONO_UltrasonicBackend backend(system, sensor_body);
    backend.set_max_distance(MAX_DISTANCE);
    backend.set_min_distance(0.5);
    system.DoStepDynamics(0.001);
    EXPECT_DOUBLE_EQ(backend.read_values().at(0), 0.5);
}

TEST(TestChronoUltrasound, TheBeamHearsObstaclesBesideTheAxis)
{
    chrono::ChSystemNSC system;
    system.SetCollisionSystemType(chrono::ChCollisionSystem::Type::BULLET);

    auto sensor_body = add_fixed_box(system, chrono::ChVector3d(0.0, 0.0, 0.0), false);

    // A small box 2 m ahead and 0.3 m to the left: about 8.5 degrees off the axis
    add_fixed_box(system, chrono::ChVector3d(2.0, 0.3, 0.0), true, 0.2);

    CHRONO_UltrasonicBackend backend(system, sensor_body);
    backend.set_max_distance(MAX_DISTANCE);
    system.DoStepDynamics(0.001);
    EXPECT_DOUBLE_EQ(backend.read_values().at(0), MAX_DISTANCE);

    // A cone 30 degrees wide, a ray every 5 degrees
    backend.set_beam(30.0 * bitrl::consts::maths::PI / 180.0, 7);
    EXPECT_NEAR(backend.read_values().at(0), 1.9 / std::cos(10.0 * bitrl::consts::maths::PI / 180.0),
                0.05);
}

TEST(TestChronoUltrasound, IgnoresTheBodyItIsAttachedTo)
{
    chrono::ChSystemNSC system;
    system.SetCollisionSystemType(chrono::ChCollisionSystem::Type::BULLET);

    // The sensor sits inside a body that collides
    auto sensor_body = add_fixed_box(system, chrono::ChVector3d(0.0, 0.0, 0.0), true);
    add_fixed_box(system, chrono::ChVector3d(2.0, 0.0, 0.0), true);

    EXPECT_NEAR(read_distance(system, sensor_body), 1.5, 1.0e-2);
}

TEST(TestChronoUltrasound, NeedsABody)
{
    CHRONO_UltrasonicBackend backend;
    EXPECT_THROW(backend.read_values(), std::logic_error);
}

TEST(TestChronoUltrasound, TheSonarOfTheRobotFileMeasuresTheDistanceAhead)
{
    using bitrl::rb::bitrl_chrono::CHRONO_DiffDriveRobot;

    chrono::ChSystemNSC system;
    system.SetCollisionSystemType(chrono::ChCollisionSystem::Type::BULLET);

    CHRONO_DiffDriveRobot robot;
    robot.load(std::filesystem::path(bitrl::consts::ROBOTS_DIR + "/bitrl_diff_drive_robot.json"));
    robot.add_to_sys(system);

    auto sonar = robot.get_sonar();
    ASSERT_TRUE(sonar);
    EXPECT_TRUE(sonar->is_enabled());
    EXPECT_EQ(sonar->backend_type_str(), "SIMULATION");
    EXPECT_DOUBLE_EQ(sonar->range_backend()->max_distance(), 4.0);
    EXPECT_DOUBLE_EQ(sonar->range_backend()->min_distance(), 0.02);
    EXPECT_EQ(robot.get_sensor_manager().get(sonar->sensor_name()), sonar);

    // Nothing ahead
    system.DoStepDynamics(0.001);
    EXPECT_DOUBLE_EQ(sonar->read_values().at(0), 4.0);

    // The sonar is 0.27 m ahead of the centre of the chassis, and the box's
    // face is at x = 1.35
    const auto chassis = robot.get_chassis()->GetPos();
    auto box =
        add_fixed_box(system, chrono::ChVector3d(chassis.x() + 1.5, 0.0, chassis.z()), true, 0.3);
    // The system has started, so a new body has to join its collision system
    system.GetCollisionSystem()->BindItem(box);
    system.DoStepDynamics(0.001);
    EXPECT_NEAR(sonar->read_values().at(0), 1.08, 0.01);
    EXPECT_NEAR(sonar->distance(), 1.08, 0.01);
}

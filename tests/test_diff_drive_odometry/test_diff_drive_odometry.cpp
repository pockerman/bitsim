#include "bitsim/bitsim_config.h"
#include "bitsim/bitsim_consts.h"
#include "bitsim/bitsim_types.h"
#include "bitsim/estimation/diff_drive_odometry.h"

#ifdef BITSIM_CHRONO
#include "bitsim/rigid_bodies/chrono_robots/diff_drive_robot.h"
#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChSystemNSC.h>
#include <filesystem>
#endif

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

namespace
{
using bitsim::real_t;
using bitsim::estimation::DiffDriveOdometry;
const real_t PI = bitsim::consts::maths::PI;
const real_t R = 0.1; // wheel radius [m]
const real_t L = 0.4; // axle length [m]
} // namespace

TEST(TestDiffDriveOdometry, EqualTurnsDriveStraight)
{
    DiffDriveOdometry odometry(R, L);
    odometry.reset({1.0, 2.0, PI / 2.0}, 0.0, 0.0);
    for (int i = 1; i <= 10; ++i)
    {
        odometry.update(i * 1.0, i * 1.0); // 0.1 m a step
    }
    EXPECT_NEAR(odometry.pose().x, 1.0, 1e-12);
    EXPECT_NEAR(odometry.pose().y, 3.0, 1e-12);
    EXPECT_NEAR(odometry.pose().yaw, PI / 2.0, 1e-12);
    EXPECT_NEAR(odometry.distance(), 1.0, 1e-12);
}

TEST(TestDiffDriveOdometry, OppositeTurnsSpinOnTheSpot)
{
    DiffDriveOdometry odometry(R, L);
    odometry.reset({}, 0.0, 0.0);
    // A quarter turn: each wheel rolls (L / 2) (pi / 2) along the ground
    const real_t wheel = 0.5 * L * 0.5 * PI / R;
    odometry.update(-wheel, wheel);
    EXPECT_NEAR(odometry.pose().yaw, PI / 2.0, 1e-12);
    EXPECT_NEAR(odometry.pose().x, 0.0, 1e-12);
    EXPECT_NEAR(odometry.distance(), 0.0, 1e-12);
}

TEST(TestDiffDriveOdometry, ACircleCloses)
{
    // Radius 1 m to the left: the wheels roll on circles of 0.8 m and 1.2 m
    DiffDriveOdometry odometry(R, L);
    odometry.reset({}, 0.0, 0.0);
    const int n = 1000;
    for (int i = 1; i <= n; ++i)
    {
        const real_t angle = 2.0 * PI * i / n;
        odometry.update(0.8 * angle / R, 1.2 * angle / R);
        if (i == n / 2)
        {
            EXPECT_NEAR(odometry.pose().x, 0.0, 1e-4);
            EXPECT_NEAR(odometry.pose().y, 2.0, 1e-4);
        }
    }
    EXPECT_NEAR(odometry.pose().x, 0.0, 1e-4);
    EXPECT_NEAR(odometry.pose().y, 0.0, 1e-4);
    EXPECT_NEAR(odometry.pose().yaw, 0.0, 1e-9);
    EXPECT_NEAR(odometry.distance(), 2.0 * PI, 1e-9);
}

TEST(TestDiffDriveOdometry, AHeadingFromElsewhereOverridesTheWheels)
{
    DiffDriveOdometry odometry(R, L);
    odometry.reset({}, 0.0, 0.0);
    // The wheels say straight ahead, the gyro says the robot faces along y
    odometry.update(1.0, 1.0, PI / 2.0);
    odometry.update(2.0, 2.0, PI / 2.0);
    EXPECT_NEAR(odometry.pose().yaw, PI / 2.0, 1e-12);
    // The first step goes along the mean of 0 and pi / 2
    EXPECT_NEAR(odometry.pose().x, 0.1 * std::cos(PI / 4.0), 1e-12);
    EXPECT_NEAR(odometry.pose().y, 0.1 * std::sin(PI / 4.0) + 0.1, 1e-12);
}

TEST(TestDiffDriveOdometry, TheHeadingWrapsAround)
{
    DiffDriveOdometry odometry(R, L);
    odometry.reset({0.0, 0.0, PI - 0.05}, 0.0, 0.0);
    odometry.update(1.0, 1.0, -PI + 0.05); // crossing +-pi
    EXPECT_NEAR(odometry.pose().x, -0.1, 1e-12);
    EXPECT_NEAR(odometry.pose().y, 0.0, 1e-12);
}

TEST(TestDiffDriveOdometry, NeedsAPositiveGeometry)
{
    EXPECT_THROW(DiffDriveOdometry(0.0, L), std::logic_error);
    EXPECT_THROW(DiffDriveOdometry(R, -1.0), std::logic_error);
}

#ifdef BITSIM_CHRONO
TEST(TestDiffDriveOdometry, TheEncodersOfTheRobotTrackItsPose)
{
    using bitsim::rb::bitsim_chrono::CHRONO_DiffDriveRobot;

    chrono::ChSystemNSC system;
    system.SetGravitationalAcceleration(chrono::ChVector3d(0.0, 0.0, -bitsim::consts::maths::G));
    system.SetCollisionSystemType(chrono::ChCollisionSystem::Type::BULLET);
    auto material = chrono_types::make_shared<chrono::ChContactMaterialNSC>();
    material->SetFriction(0.8f);
    auto ground =
        chrono_types::make_shared<chrono::ChBodyEasyBox>(20.0, 20.0, 0.1, 1000.0, false, true, material);
    ground->SetPos(chrono::ChVector3d(0.0, 0.0, -0.05));
    ground->SetFixed(true);
    system.Add(ground);

    CHRONO_DiffDriveRobot robot;
    robot.load(std::filesystem::path(bitsim::consts::ROBOTS_DIR + "/bitsim_diff_drive_robot.json"));
    robot.add_to_sys(system);

    const auto &data = robot.get_data();
    DiffDriveOdometry odometry(data.wheel_radius, data.axle_length);
    auto left = robot.get_motor(CHRONO_DiffDriveRobot::LEFT);
    auto right = robot.get_motor(CHRONO_DiffDriveRobot::RIGHT);
    const auto start = robot.axle_center();
    odometry.reset({start.x(), start.y(), robot.heading()}, left->encoder_angle(),
                   right->encoder_angle());

    // An arc to the left for 4 s
    robot.set_velocity(0.3, 0.4);
    for (int i = 0; i < 400; ++i)
    {
        robot.update(0.01);
        system.DoStepDynamics(0.01);
        odometry.update(left->encoder_angle(), right->encoder_angle());
    }

    // The encoders count whole steps of 2 pi / 660, and the wheels slip a
    // little, so odometry is close but not exact
    const auto end = robot.axle_center();
    EXPECT_GT(std::hypot(end.x() - start.x(), end.y() - start.y()), 0.8);
    EXPECT_NEAR(odometry.pose().x, end.x(), 0.03);
    EXPECT_NEAR(odometry.pose().y, end.y(), 0.03);
    EXPECT_NEAR(std::remainder(odometry.pose().yaw - robot.heading(), 2.0 * PI), 0.0, 0.05);

    // The encoder counts whole steps
    const real_t step = 2.0 * PI / 660.0;
    const real_t counts = left->encoder_angle() / step;
    EXPECT_NEAR(counts, std::round(counts), 1e-9);
}
#endif

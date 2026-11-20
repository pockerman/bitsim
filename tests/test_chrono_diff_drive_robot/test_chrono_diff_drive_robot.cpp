#include "bitsim/actuators/backends/dc_motor_backend_base.h"
#include "bitsim/actuators/dc_motor.h"
#include "bitsim/bitsim_config.h"
#include "bitsim/bitsim_consts.h"
#include "bitsim/bitsim_types.h"
#include "bitsim/rigid_bodies/chrono_robots/diff_drive_robot.h"

#ifdef BITSIM_OPENCV
#include "bitsim/sensors/downward_camera.h"
#endif

#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChContactMaterialNSC.h>
#include <chrono/physics/ChSystemNSC.h>

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <memory>
#include <stdexcept>

namespace
{

using bitsim::real_t;
using bitsim::actuators::DCMotor;
using bitsim::rb::bitsim_chrono::CHRONO_DiffDriveRobot;

const real_t STEP_SIZE = 0.01;

// A motor that turns at a fixed speed, as a real one on a board would
class TurningBackend : public bitsim::actuators::backends::DCMotorBackendBase
{
  public:
    explicit TurningBackend(real_t speed) : DCMotorBackendBase("TURNING"), speed_(speed) {}

    OutputType output_type() const noexcept override { return OutputType::ANGLE; }
    void set_target_speed(real_t) override {}
    void step(real_t dt) override { angle_ += speed_ * dt; }
    void step(real_t dt, const bitsim::actuators::ShaftState &) override { step(dt); }
    real_t angle() const override { return angle_; }
    real_t speed() const override { return speed_; }

  private:
    real_t speed_;
    real_t angle_ = 0.0;
};

class TestChronoDiffDriveRobot : public ::testing::Test
{
  protected:
    void SetUp() override
    {
        system_.SetGravitationalAcceleration(chrono::ChVector3d(0.0, 0.0, -9.81));
        system_.SetCollisionSystemType(chrono::ChCollisionSystem::Type::BULLET);
        system_.SetSolverType(chrono::ChSolver::Type::BARZILAIBORWEIN);
        system_.GetSolver()->AsIterative()->SetMaxIterations(40);

        auto material = chrono_types::make_shared<chrono::ChContactMaterialNSC>();
        material->SetFriction(0.8f);
        auto floor = chrono_types::make_shared<chrono::ChBodyEasyBox>(10.0, 10.0, 0.1, 1000.0,
                                                                      false, true, material);
        floor->SetPos(chrono::ChVector3d(0.0, 0.0, -0.05));
        floor->SetFixed(true);
        system_.Add(floor);

        robot_.load(
            std::filesystem::path(bitsim::consts::ROBOTS_DIR + "/bitsim_diff_drive_robot.json"));
    }

    void run(real_t duration)
    {
        for (int i = 0; i < static_cast<int>(std::round(duration / STEP_SIZE)); ++i)
        {
            robot_.update(STEP_SIZE);
            system_.DoStepDynamics(STEP_SIZE);
        }
    }

    real_t heading() const
    {
        const auto forward = robot_.get_pose().rotation_matrix() * chrono::ChVector3d(1, 0, 0);
        return std::atan2(forward.y(), forward.x());
    }

    chrono::ChSystemNSC system_;
    CHRONO_DiffDriveRobot robot_;
};

} // namespace

TEST(TestChronoDiffDriveRobotLoad, NeedsToBeLoaded)
{
    CHRONO_DiffDriveRobot robot;
    chrono::ChSystemNSC system;
    EXPECT_THROW(robot.add_to_sys(system), std::logic_error);
    EXPECT_THROW(robot.set_motor_speed(0, 1.0), std::logic_error);
}

TEST_F(TestChronoDiffDriveRobot, HasSimulatedMotorsFromTheRobotFile)
{
    EXPECT_EQ(robot_.n_wheel_motors(), 2u);
    for (bitsim::uint_t m = 0; m < 2; ++m)
    {
        EXPECT_EQ(robot_.get_motor(m)->output_type(), DCMotor::OutputType::TORQUE);
    }
    EXPECT_THROW(robot_.get_motor(2), std::logic_error);
}

TEST_F(TestChronoDiffDriveRobot, DrivesStraightAhead)
{
    robot_.add_to_sys(system_);
    const auto start = robot_.axle_center();

    robot_.set_speed(0.25);
    run(4.0);

    const auto moved = robot_.axle_center() - start;
    EXPECT_GT(moved.x(), 0.6);
    EXPECT_LT(std::abs(moved.y()), 0.02);
    EXPECT_LT(std::abs(heading()), 2.0 * bitsim::consts::maths::PI / 180.0);

    // Without slip the wheels covered the same distance as the robot
    const real_t r = robot_.get_data().wheel_radius;
    EXPECT_NEAR(r * robot_.wheel_angle(0), moved.x(), 0.02);
    EXPECT_NEAR(r * robot_.wheel_angle(1), moved.x(), 0.02);
}

TEST_F(TestChronoDiffDriveRobot, TurnsLeftOnTheSpot)
{
    robot_.add_to_sys(system_);
    const auto start = robot_.axle_center();

    robot_.set_velocity(0.0, 0.5);
    run(2.0);

    EXPECT_GT(heading(), 0.3);
    EXPECT_LT((robot_.axle_center() - start).Length(), 0.03);
}

TEST_F(TestChronoDiffDriveRobot, WheelsFollowMotorsThatReportAnAngle)
{
    robot_.add_to_sys(system_);

    // Swapping the motors after add_to_sys reconnects the wheels
    const real_t speed = 2.0;
    for (bitsim::uint_t m = 0; m < 2; ++m)
    {
        robot_.set_motor(m, std::make_shared<DCMotor>(std::make_shared<TurningBackend>(speed)));
    }
    const auto start = robot_.axle_center();
    run(2.0);

    const real_t r = robot_.get_data().wheel_radius;
    EXPECT_NEAR(robot_.wheel_angle(0), speed * 2.0, 0.05);
    EXPECT_NEAR(robot_.wheel_angle(1), speed * 2.0, 0.05);
    EXPECT_NEAR((robot_.axle_center() - start).x(), r * speed * 2.0, 0.03);
}

TEST_F(TestChronoDiffDriveRobot, CanBePlacedAnywhereFacingAnyWay)
{
    const real_t yaw = bitsim::consts::maths::PI / 2.0;
    robot_.set_world_pose(1.0, -0.5, yaw);
    EXPECT_NEAR(robot_.axle_center().x(), 1.0, 1e-9);
    EXPECT_NEAR(robot_.axle_center().y(), -0.5, 1e-9);
    EXPECT_NEAR(robot_.heading(), yaw, 1e-9);

    // It drives the way it faces, so the joints moved with it
    robot_.add_to_sys(system_);
    robot_.set_speed(0.25);
    run(2.0);
    const auto moved = robot_.axle_center() - chrono::ChVector3d(1.0, -0.5, 0.0);
    EXPECT_GT(moved.y(), 0.3);
    EXPECT_LT(std::abs(moved.x()), 0.03);
    EXPECT_NEAR(robot_.heading(), yaw, 0.03);
}

#ifdef BITSIM_OPENCV
TEST_F(TestChronoDiffDriveRobot, HasTheCameraOfTheRobotFile)
{
    ASSERT_TRUE(robot_.get_camera());
    const auto &config = robot_.get_camera()->config();

    // A floor 2 m wide, white with a black stripe across the middle of the
    // camera's view, offset ahead of the axle centre
    cv::Mat image(200, 200, CV_8UC1, cv::Scalar(255));
    const bitsim::sensors::FloorImage floor(image, 2.0);
    const real_t stripe_x = robot_.axle_center().x() + config.offset;
    const int column = static_cast<int>(std::round(floor.to_pixel(stripe_x, 0.0).x));
    image.colRange(column - 2, column + 3).setTo(cv::Scalar(0));

    const cv::Mat view = robot_.camera_view(floor);
    EXPECT_EQ(view.cols, static_cast<int>(config.image_width));

    // Facing +x the stripe crosses the middle row of the view from left to right
    const cv::Mat middle_row = view.row(view.rows / 2);
    EXPECT_EQ(cv::countNonZero(middle_row < 128), view.cols);
}
#endif

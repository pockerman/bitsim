#include "bitsim/bitsim_consts.h"
#include "bitsim/bitsim_types.h"
#include "bitsim/rigid_bodies/chrono_robots/diff_drive_robot.h"
#include "bitsim/sensors/backends/chrono_imu_backend.h"
#include "bitsim/sensors/backends/imu_backend_base.h"
#include "bitsim/sensors/imu_sensor.h"
#include "bitsim/sensors/messages/imu.h"

#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChSystemNSC.h>

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <memory>
#include <stdexcept>

namespace
{
using bitsim::real_t;
using bitsim::sensors::IMUMessage;
using bitsim::sensors::IMUSensor;
using bitsim::sensors::backends::CHRONO_IMUBackend;
using bitsim::sensors::backends::IMUBackendBase;

const real_t G = bitsim::consts::maths::G;

struct World
{
    chrono::ChSystemNSC system;
    std::shared_ptr<chrono::ChBody> body = chrono_types::make_shared<chrono::ChBody>();

    World()
    {
        system.SetGravitationalAcceleration(chrono::ChVector3d(0.0, 0.0, -G));
        system.Add(body);
    }
};

} // namespace

TEST(TestIMUSensor, AStillIMUFeelsGravityUpwards)
{
    World world;
    world.body->SetFixed(true);
    CHRONO_IMUBackend imu(world.body);

    const auto v = imu.read_values();
    ASSERT_EQ(v.size(), 6u);
    EXPECT_NEAR(v[0], 0.0, 1e-12);
    EXPECT_NEAR(v[1], 0.0, 1e-12);
    EXPECT_NEAR(v[2], G, 1e-12);
    for (int i = 3; i < 6; ++i)
    {
        EXPECT_NEAR(v[i], 0.0, 1e-12);
    }
}

TEST(TestIMUSensor, ReadsAlongTheAxesOfTheBody)
{
    // Rolled 90 degrees about x: the body's y axis points up
    World world;
    world.body->SetFixed(true);
    world.body->SetRot(chrono::QuatFromAngleX(bitsim::consts::maths::PI / 2.0));
    CHRONO_IMUBackend imu(world.body);

    const auto v = imu.read_values();
    EXPECT_NEAR(v[1], G, 1e-9);
    EXPECT_NEAR(v[2], 0.0, 1e-9);
}

TEST(TestIMUSensor, ASpinningIMUReadsItsRateAndTheCentripetalAcceleration)
{
    World world;
    world.body->SetAngVelParent(chrono::ChVector3d(0.0, 0.0, 2.0));
    CHRONO_IMUBackend imu(world.body);
    imu.set_position({0.5, 0.0, 0.0});

    const auto v = imu.read_values();
    EXPECT_NEAR(v[5], 2.0, 1e-12);
    // omega^2 r towards the axis, as the body has no other acceleration yet
    EXPECT_NEAR(v[0], -2.0 * 2.0 * 0.5, 1e-9);
    EXPECT_NEAR(v[2], G, 1e-9);
}

TEST(TestIMUSensor, TheGyroHasItsBiasAndNoise)
{
    World world;
    world.body->SetFixed(true);
    CHRONO_IMUBackend imu(world.body);
    CHRONO_IMUBackend::Errors errors;
    errors.gyro_bias = {0.0, 0.0, 0.01};
    errors.gyro_noise = 0.002;
    errors.accel_noise = 0.05;
    imu.set_errors(errors);

    const int n = 20000;
    real_t sum = 0.0;
    real_t sum2 = 0.0;
    real_t accel_sum2 = 0.0;
    for (int i = 0; i < n; ++i)
    {
        const auto v = imu.read_values();
        sum += v[5];
        sum2 += (v[5] - 0.01) * (v[5] - 0.01);
        accel_sum2 += (v[2] - G) * (v[2] - G);
    }
    EXPECT_NEAR(sum / n, 0.01, 1e-4);
    EXPECT_NEAR(std::sqrt(sum2 / n), 0.002, 1e-4);
    EXPECT_NEAR(std::sqrt(accel_sum2 / n), 0.05, 2e-3);

    // The same seed, the same readings
    CHRONO_IMUBackend again(world.body);
    again.set_errors(errors);
    imu.set_errors(errors);
    EXPECT_EQ(imu.read_values(), again.read_values());

    errors.gyro_noise = -1.0;
    EXPECT_THROW(imu.set_errors(errors), std::logic_error);
}

TEST(TestIMUSensor, NeedsABodyInASystem)
{
    CHRONO_IMUBackend imu;
    EXPECT_THROW(imu.read_values(), std::logic_error);
    imu.set_chrono_body(chrono_types::make_shared<chrono::ChBody>());
    EXPECT_THROW(imu.read_values(), std::logic_error);
}

TEST(TestIMUSensor, PicksOutAccelerationAndAngularVelocity)
{
    World world;
    world.body->SetFixed(true);
    IMUSensor sensor(std::make_shared<CHRONO_IMUBackend>(world.body), "imu");
    sensor.init();
    EXPECT_EQ(sensor.angular_velocity()[2], 0.0);
    sensor.read_values();
    EXPECT_NEAR(sensor.acceleration()[2], G, 1e-12);
    EXPECT_EQ(IMUBackendBase::create("SIMULATION")->backend_type_str(), "SIMULATION");
    EXPECT_THROW(IMUBackendBase::create("NO_SUCH_BACKEND"), std::runtime_error);
}

TEST(TestIMUSensor, LoadsItsSettingsFromJson)
{
    CHRONO_IMUBackend imu;
    imu.load(nlohmann::json::parse(R"({"frame_pos": [0.1, 0.0, 0.05], "update_rate": 200,
        "gyro_bias": [0, 0, 0.003], "gyro_noise": 0.002, "accel_noise": 0.04, "seed": 7})"));
    EXPECT_DOUBLE_EQ(imu.position()[0], 0.1);
    EXPECT_DOUBLE_EQ(imu.sampling_period(), 0.005);
    EXPECT_DOUBLE_EQ(imu.errors().gyro_bias[2], 0.003);
    EXPECT_DOUBLE_EQ(imu.errors().accel_noise, 0.04);
    EXPECT_EQ(imu.errors().seed, 7u);
}

TEST(TestIMUSensor, ParsesMessages)
{
    const auto m = IMUMessage::parse(R"({"accel": [0.1, 0.2, 9.8], "gyro": [0, 0, 0.35]})");
    ASSERT_TRUE(m);
    EXPECT_DOUBLE_EQ(m->accel[2], 9.8);
    EXPECT_DOUBLE_EQ(m->gyro[2], 0.35);
    EXPECT_FALSE(IMUMessage::parse(R"({"accel": [0.1, 0.2], "gyro": [0, 0, 0]})"));
    EXPECT_FALSE(IMUMessage::parse(R"({"accel": [0.1, 0.2, 9.8]})"));
    EXPECT_FALSE(IMUMessage::parse(R"({"accel": [0, 0, "up"], "gyro": [0, 0, 0]})"));
    EXPECT_FALSE(IMUMessage::parse("nonsense"));
}

TEST(TestIMUSensor, TheIMUOfTheRobotFileFeelsTheRobotTurn)
{
    using bitsim::rb::bitsim_chrono::CHRONO_DiffDriveRobot;

    chrono::ChSystemNSC system;
    system.SetGravitationalAcceleration(chrono::ChVector3d(0.0, 0.0, -G));
    system.SetCollisionSystemType(chrono::ChCollisionSystem::Type::BULLET);
    auto ground = chrono_types::make_shared<chrono::ChBodyEasyBox>(
        10.0, 10.0, 0.1, 1000.0, false, true,
        chrono_types::make_shared<chrono::ChContactMaterialNSC>());
    ground->SetPos(chrono::ChVector3d(0.0, 0.0, -0.05));
    ground->SetFixed(true);
    system.Add(ground);

    CHRONO_DiffDriveRobot robot;
    robot.load(std::filesystem::path(bitsim::consts::ROBOTS_DIR + "/bitsim_diff_drive_robot.json"));
    robot.add_to_sys(system);

    auto imu = robot.get_imu();
    ASSERT_TRUE(imu);
    EXPECT_TRUE(imu->is_enabled());
    EXPECT_EQ(imu->backend_type_str(), "SIMULATION");

    // Turn on the spot, and average the gyro and the true rate over the last
    // second. The gyro reads the true rate plus its bias
    robot.set_velocity(0.0, 1.0);
    real_t sum = 0.0;
    real_t true_sum = 0.0;
    int n = 0;
    for (int i = 0; i < 300; ++i)
    {
        robot.update(0.01);
        system.DoStepDynamics(0.01);
        if (i >= 200)
        {
            sum += imu->read_values()[5];
            true_sum += robot.get_chassis()->GetAngVelLocal().z();
            ++n;
        }
    }
    EXPECT_NEAR(true_sum / n, 1.0, 0.1);
    EXPECT_NEAR(sum / n, true_sum / n + 0.003, 0.001);
    EXPECT_NEAR(imu->acceleration()[2], G, 1.0);
}

#include "bitrl/bitrl_consts.h"
#include "bitrl/bitrl_types.h"
#include "bitrl/estimation/diff_drive_ekf.h"
#include "bitrl/trajectory/rounded_rectangle.h"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <vector>

namespace
{
using bitrl::real_t;
using bitrl::estimation::Box2d;
using bitrl::estimation::DiffDriveEKF;
using bitrl::estimation::DiffDriveMap;
using bitrl::trajectory::RoundedRectangle;

const real_t PI = bitrl::consts::maths::PI;

// The track of the line following examples in a room of 6 x 5 m
DiffDriveMap room()
{
    const real_t t = 0.1;
    return DiffDriveMap(RoundedRectangle(2.2, 1.4, 0.7), {{-3.0 - t, -2.5, -3.0, 2.5},
                                                          {3.0, -2.5, 3.0 + t, 2.5},
                                                          {-3.0, -2.5 - t, 3.0, -2.5},
                                                          {-3.0, 2.5, 3.0, 2.5 + t}});
}

const DiffDriveEKF::RangeSensor FRONT{0.37, 0.0, 0.0, 0.0, 1, 0.02, 4.0};

// Where the line is seen from a robot at (x, y) heading 0 on the bottom straight, y = -1.4,
// at the given distances ahead
std::vector<std::array<real_t, 2>> line_seen(real_t y, const std::vector<real_t> &ahead)
{
    std::vector<std::array<real_t, 2>> points;
    for (const real_t a : ahead)
    {
        points.push_back({a, -1.4 - y});
    }
    return points;
}
} // namespace

TEST(TestDiffDriveMap, RayCastAndSignedDistance)
{
    const auto map = room();
    EXPECT_NEAR(map.ray_cast(0.0, 0.0, 0.0, 10.0), 3.0, 1.0e-12);
    EXPECT_NEAR(map.ray_cast(0.0, 0.0, PI / 2, 10.0), 2.5, 1.0e-12);
    EXPECT_NEAR(map.ray_cast(0.0, 0.0, PI, 10.0), 3.0, 1.0e-12);
    EXPECT_NEAR(map.ray_cast(0.0, 0.0, std::atan2(2.5, 3.0), 10.0), std::hypot(3.0, 2.5), 1.0e-9);
    EXPECT_NEAR(map.ray_cast(0.0, 0.0, 0.0, 2.0), 2.0, 1.0e-12); // out of range

    const auto &track = map.track();
    EXPECT_NEAR(track.signed_distance(0.0, -1.4), 0.0, 1.0e-12);
    EXPECT_NEAR(track.signed_distance(0.0, -1.3), -0.1, 1.0e-12); // inside
    EXPECT_NEAR(track.signed_distance(0.0, -1.5), 0.1, 1.0e-12);  // outside
}

TEST(TestDiffDriveEKF, PredictionFollowsTheEncodersAndTheGyro)
{
    DiffDriveEKF ekf(room(), {0.0, 0.0, 0.0}, {});
    // A quarter circle of radius 1: 1 m/s turning at 1 rad/s for pi/2 s
    const real_t dt = 0.001;
    const int n = static_cast<int>(std::round(PI / 2 / dt));
    for (int k = 0; k < n; ++k)
    {
        ekf.predict(dt, 1.0, dt);
    }
    EXPECT_NEAR(ekf.pose().x, 1.0, 1.0e-3);
    EXPECT_NEAR(ekf.pose().y, 1.0, 1.0e-3);
    EXPECT_NEAR(ekf.pose().yaw, PI / 2, 1.0e-3);
    EXPECT_GT(ekf.covariance()(0, 0), 0.02 * 0.02); // dead reckoning grows the uncertainty
}

TEST(TestDiffDriveEKF, TheLineKeepsTheHeadingAndTheFilterLearnsTheGyroBias)
{
    // The robot drives along the bottom straight at 0.3 m/s; the gyroscope reads a bias of
    // 0.01 rad/s that the filter does not know. The camera sees the line at 0.25 and 0.45 m ahead
    DiffDriveEKF ekf(room(), {-1.5, -1.4, 0.0}, {});
    const real_t dt = 0.01;
    const real_t bias = 0.01;
    real_t x = -1.5;
    for (int k = 0; k < 1000; ++k)
    {
        x += 0.3 * dt;
        ekf.predict(0.3 * dt, 0.0 + bias, dt);
        if (k % 5 == 0)
        {
            ekf.update_line(line_seen(-1.4, {0.25, 0.45}), 0.01);
        }
    }
    EXPECT_NEAR(ekf.pose().yaw, 0.0, 0.01);
    EXPECT_NEAR(ekf.pose().y, -1.4, 0.01);
    EXPECT_NEAR(ekf.gyro_bias(), bias, 0.003);
    (void)x;
}

TEST(TestDiffDriveEKF, RangesAheadFixThePositionAlongTheStraight)
{
    // The filter starts 0.3 m behind where the robot is; the front sonar ranges the wall ahead
    DiffDriveEKF ekf(room(), {0.2, -1.4, 0.0}, {}, {0.3, 0.02, 0.005});
    const real_t true_x = 0.5;
    for (int k = 0; k < 20; ++k)
    {
        ekf.predict(0.0, 0.0, 0.05);
        const real_t measured = 3.0 - (true_x + FRONT.x);
        const auto result = ekf.update_range(FRONT, measured);
        EXPECT_TRUE(result.accepted);
    }
    EXPECT_NEAR(ekf.pose().x, true_x, 0.02);
}

TEST(TestDiffDriveEKF, TheGateRejectsAnEchoTheMapCannotExplain)
{
    DiffDriveEKF ekf(room(), {-1.5, -1.4, 0.0}, {});
    // The map says 3.0 - (-1.5 + 0.37) = 4.13: out of range, so no update at all
    EXPECT_FALSE(ekf.update_range(FRONT, 1.0).accepted);

    // Nearer the wall the map predicts 1.63 m; a box 1 m ahead, not on the map, is rejected
    DiffDriveEKF near(room(), {1.0, -1.4, 0.0}, {});
    const auto rejected = near.update_range(FRONT, 1.0);
    EXPECT_FALSE(rejected.accepted);
    EXPECT_GT(rejected.mahalanobis, 9.0);
    EXPECT_NEAR(near.pose().x, 1.0, 1.0e-12);

    // A reading of no echo is not used either
    EXPECT_FALSE(near.update_range(FRONT, FRONT.max_range).accepted);
}

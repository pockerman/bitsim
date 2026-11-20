#include "bitsim/bitsim_consts.h"
#include "bitsim/bitsim_types.h"
#include "bitsim/geometry/rotations.h"

#include <chrono/core/ChMatrix33.h>
#include <chrono/core/ChVector3.h>
#include <gtest/gtest.h>

namespace
{
using bitsim::real_t;
using chrono::ChMatrix33d;
using chrono::ChVector3d;
using namespace bitsim::geometry;

const real_t DEG = bitsim::consts::maths::PI / 180.0;
const real_t TOL = 1.0e-12;
} // namespace

TEST(TestRotations, ElementaryRotationsMatchChrono)
{
    EXPECT_TRUE(same_matrix(rot_x(30.0 * DEG), ChMatrix33d(30.0 * DEG, ChVector3d(1, 0, 0))));
    EXPECT_TRUE(same_matrix(rot_y(30.0 * DEG), ChMatrix33d(30.0 * DEG, ChVector3d(0, 1, 0))));
    EXPECT_TRUE(same_matrix(rot_z(30.0 * DEG), ChMatrix33d(30.0 * DEG, ChVector3d(0, 0, 1))));
}

TEST(TestRotations, RotZTurnsXIntoY)
{
    const ChVector3d v = rot_z(90.0 * DEG) * ChVector3d(1.0, 0.0, 0.0);
    EXPECT_NEAR(v.x(), 0.0, TOL);
    EXPECT_NEAR(v.y(), 1.0, TOL);
    EXPECT_NEAR(v.z(), 0.0, TOL);
}

TEST(TestRotations, IsRotation)
{
    const ChMatrix33d R = rot_z(20.0 * DEG) * rot_y(-35.0 * DEG) * rot_x(70.0 * DEG);
    EXPECT_TRUE(is_rotation(R));

    // orthogonal but left-handed
    ChMatrix33d mirror = R;
    mirror.col(2) *= -1.0;
    EXPECT_FALSE(is_rotation(mirror));

    // right-handed but it scales
    const ChMatrix33d scaled = 2.0 * R;
    EXPECT_FALSE(is_rotation(scaled));
}

TEST(TestRotations, YawPitchRollRoundTrip)
{
    const YawPitchRoll angles{40.0 * DEG, -25.0 * DEG, 60.0 * DEG};
    const ChMatrix33d R = from_yaw_pitch_roll(angles);
    EXPECT_TRUE(same_matrix(R, rot_z(angles.yaw) * rot_y(angles.pitch) * rot_x(angles.roll)));

    const YawPitchRoll back = to_yaw_pitch_roll(R);
    EXPECT_NEAR(back.yaw, angles.yaw, TOL);
    EXPECT_NEAR(back.pitch, angles.pitch, TOL);
    EXPECT_NEAR(back.roll, angles.roll, TOL);
}

TEST(TestRotations, GimbalLockLosesYawMinusRoll)
{
    // At a pitch of 90 degrees only yaw - roll matters, so two different triples give the
    // same matrix
    const ChMatrix33d A = from_yaw_pitch_roll({30.0 * DEG, 90.0 * DEG, 10.0 * DEG});
    const ChMatrix33d B = from_yaw_pitch_roll({50.0 * DEG, 90.0 * DEG, 30.0 * DEG});
    EXPECT_TRUE(same_matrix(A, B, 1.0e-9));
}

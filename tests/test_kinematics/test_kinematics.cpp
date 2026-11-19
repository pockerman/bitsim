#include "bitrl/bitrl_consts.h"
#include "bitrl/bitrl_types.h"
#include "bitrl/kinematics/denavit_hartenberg.h"
#include "bitrl/kinematics/planar_arm.h"
#include "bitrl/kinematics/spatial_arm.h"

#include <gtest/gtest.h>

#include <cmath>

namespace
{
using bitrl::real_t;
using chrono::ChVector3d;
using namespace bitrl::kinematics;

const real_t DEG = bitrl::consts::maths::PI / 180.0;
const real_t TOL = 1.0e-9;

void expect_near(const ChVector3d &a, const ChVector3d &b, real_t tol = TOL)
{
    EXPECT_NEAR(a.x(), b.x(), tol);
    EXPECT_NEAR(a.y(), b.y(), tol);
    EXPECT_NEAR(a.z(), b.z(), tol);
}
} // namespace

TEST(TestPlanarArm, FormulaAndFramesAgree)
{
    const PlanarArm arm;
    for (const PlanarJoints q : {PlanarJoints{0.0, 0.0}, PlanarJoints{30.0 * DEG, 45.0 * DEG},
                                 PlanarJoints{-120.0 * DEG, 80.0 * DEG}})
    {
        expect_near(planar_fk(arm, q), planar_fk_frames(arm, q).GetPos());
    }
    expect_near(planar_fk(arm, {0.0, 0.0}), {0.7, 0.0, 0.0});
}

TEST(TestPlanarArm, InverseKinematicsHasTwoSolutions)
{
    const PlanarArm arm;
    const auto solutions = planar_ik(arm, 0.4, 0.3);
    ASSERT_EQ(solutions.size(), 2u);
    EXPECT_GT(solutions[0][1], 0.0);
    EXPECT_LT(solutions[1][1], 0.0);
    for (const auto &q : solutions)
    {
        expect_near(planar_fk(arm, q), {0.4, 0.3, 0.0});
    }
}

TEST(TestPlanarArm, StretchedHasOneSolutionAndOutOfReachNone)
{
    const PlanarArm arm;
    EXPECT_EQ(planar_ik(arm, 0.7, 0.0).size(), 1u);
    EXPECT_TRUE(planar_ik(arm, 0.8, 0.0).empty());
    EXPECT_TRUE(planar_ik(arm, 0.05, 0.0).empty());
}

TEST(TestPlanarArm, JacobianMatchesFiniteDifferences)
{
    const PlanarArm arm;
    const PlanarJoints q{20.0 * DEG, 50.0 * DEG};
    const auto J = planar_jacobian(arm, q);
    const real_t h = 1.0e-6;
    for (int j = 0; j < 2; ++j)
    {
        PlanarJoints plus = q;
        PlanarJoints minus = q;
        plus[j] += h;
        minus[j] -= h;
        const ChVector3d d = (planar_fk(arm, plus) - planar_fk(arm, minus)) / (2.0 * h);
        EXPECT_NEAR(J(0, j), d.x(), 1.0e-7);
        EXPECT_NEAR(J(1, j), d.y(), 1.0e-7);
    }
    EXPECT_NEAR(planar_jacobian(arm, {10.0 * DEG, 0.0}).determinant(), 0.0, TOL);
}

TEST(TestSpatialArm, HomePose)
{
    const SpatialArm arm;
    const auto frames = spatial_fk_all(arm, {0.0, 0.0, 0.0});
    ASSERT_EQ(frames.size(), 4u);
    expect_near(frames[0].GetPos(), {0.0, 0.0, 0.3});
    expect_near(frames.back().GetPos(), {0.7, 0.0, 0.3});
}

TEST(TestSpatialArm, InverseKinematicsSolutionsReachTheTarget)
{
    const SpatialArm arm;
    const ChVector3d target(0.3, 0.2, 0.5);
    const auto solutions = spatial_ik(arm, target);
    ASSERT_EQ(solutions.size(), 4u);
    for (const auto &q : solutions)
    {
        expect_near(spatial_fk(arm, q).GetPos(), target);
    }

    const auto up = spatial_ik_elbow_up(arm, target);
    ASSERT_TRUE(up.has_value());
    EXPECT_LE((*up)[2], 0.0);
    EXPECT_NEAR((*up)[0], std::atan2(target.y(), target.x()), TOL);

    EXPECT_TRUE(spatial_ik(arm, {2.0, 0.0, 0.3}).empty());
    EXPECT_FALSE(spatial_ik_elbow_up(arm, {2.0, 0.0, 0.3}).has_value());
}

TEST(TestSpatialArm, DenavitHartenbergAgreesWithFrames)
{
    const SpatialArm arm;
    const SpatialJoints q{25.0 * DEG, 40.0 * DEG, -70.0 * DEG};
    const auto X_frames = spatial_fk(arm, q);
    const auto X_dh = dh_fk(spatial_dh_table(arm, q));
    expect_near(X_frames.GetPos(), X_dh.GetPos());
    EXPECT_LT((X_frames.GetRotMat() - X_dh.GetRotMat()).norm(), TOL);
}

TEST(TestSpatialArm, NumericalIKConverges)
{
    const SpatialArm arm;
    const ChVector3d target(0.35, -0.25, 0.45);
    int calls = 0;
    const auto result = numerical_ik(arm, target, {0.1, 0.3, -0.5}, 0.0, 100, 1.0e-6, 0.5,
                                     [&calls](int, const SpatialJoints &, real_t) { ++calls; });
    EXPECT_TRUE(result.converged);
    EXPECT_LT(result.error, 1.0e-6);
    EXPECT_EQ(calls, result.iterations + 1);
    expect_near(spatial_fk(arm, result.q).GetPos(), target, 1.0e-6);
}

TEST(TestSpatialArm, NumericalIKOutOfReachStopsAtTheBest)
{
    const SpatialArm arm;
    const auto result = numerical_ik(arm, {1.5, 0.0, 0.3}, {0.0, 0.2, -0.2}, 0.1);
    // The best the arm can do is to stretch towards the target, 0.8 m short of it
    EXPECT_FALSE(result.converged);
    EXPECT_GE(result.error, 0.8 - 1.0e-9);
    EXPECT_LT(result.error, 0.82);
}

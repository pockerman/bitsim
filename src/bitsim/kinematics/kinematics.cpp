#include "bitsim/bitsim_config.h"

#ifdef BITSIM_CHRONO

#include "bitsim/bitsim_consts.h"
#include "bitsim/geometry/rotations.h"
#include "bitsim/kinematics/denavit_hartenberg.h"
#include "bitsim/kinematics/joint.h"
#include "bitsim/kinematics/planar_arm.h"
#include "bitsim/kinematics/spatial_arm.h"
#include "bitsim/maths/math_utils.h"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <utility>

namespace bitsim
{
namespace kinematics
{

using chrono::ChFrame;
using chrono::ChVector3d;
using geometry::rot_x;
using geometry::rot_z;
using maths::wrap_angle;

namespace
{
const real_t PI = consts::maths::PI;
const real_t QUARTER_TURN = PI / 2.0;
} // namespace

ChFrame<> joint_rotation(real_t q) { return ChFrame<>(ChVector3d(0.0, 0.0, 0.0), rot_z(q)); }

// ---------------------------------------------------------------------------
// PlanarArm
// ---------------------------------------------------------------------------

ChVector3d planar_fk(const PlanarArm &arm, const PlanarJoints &q)
{
    return {arm.l1 * std::cos(q[0]) + arm.l2 * std::cos(q[0] + q[1]),
            arm.l1 * std::sin(q[0]) + arm.l2 * std::sin(q[0] + q[1]), 0.0};
}

ChFrame<> planar_fk_frames(const PlanarArm &arm, const PlanarJoints &q)
{
    const ChFrame<> X_W1 = joint_rotation(q[0]);
    const ChFrame<> X_12 = ChFrame<>(ChVector3d(arm.l1, 0.0, 0.0)) * joint_rotation(q[1]);
    const ChFrame<> X_2E = ChFrame<>(ChVector3d(arm.l2, 0.0, 0.0));
    return X_W1 * X_12 * X_2E;
}

std::vector<PlanarJoints> planar_ik(const PlanarArm &arm, real_t x, real_t y)
{
    // The law of cosines in the triangle shoulder, elbow, tip
    const real_t c2 = (x * x + y * y - arm.l1 * arm.l1 - arm.l2 * arm.l2) / (2.0 * arm.l1 * arm.l2);
    if (c2 < -1.0 - 1.0e-12 || c2 > 1.0 + 1.0e-12)
    {
        return {};
    }

    std::vector<PlanarJoints> solutions;
    for (const real_t sign : {1.0, -1.0})
    {
        const real_t q2 = sign * std::acos(std::clamp(c2, -1.0, 1.0));

        // The direction to the tip, minus the angle between the upper arm and that direction
        const real_t q1 =
            std::atan2(y, x) - std::atan2(arm.l2 * std::sin(q2), arm.l1 + arm.l2 * std::cos(q2));
        solutions.push_back({wrap_angle(q1), q2});
    }

    if (std::abs(wrap_angle(solutions[0][1] - solutions[1][1])) < 1.0e-6)
    {
        solutions.pop_back();
    }
    return solutions;
}

Eigen::Matrix2d planar_jacobian(const PlanarArm &arm, const PlanarJoints &q)
{
    const real_t s1 = std::sin(q[0]);
    const real_t c1 = std::cos(q[0]);
    const real_t s12 = std::sin(q[0] + q[1]);
    const real_t c12 = std::cos(q[0] + q[1]);

    Eigen::Matrix2d J;
    J << -arm.l1 * s1 - arm.l2 * s12, -arm.l2 * s12, //
        arm.l1 * c1 + arm.l2 * c12, arm.l2 * c12;
    return J;
}

// ---------------------------------------------------------------------------
// SpatialArm
// ---------------------------------------------------------------------------

std::vector<ChFrame<>> spatial_fk_all(const SpatialArm &arm, const SpatialJoints &q)
{
    // Joint 1 sits on the base and turns about the vertical axis
    const ChFrame<> X_W1 = ChFrame<>(ChVector3d(0.0, 0.0, arm.h0)) * joint_rotation(q[0]);

    // The shoulder turns about a horizontal axis. Rx(90) brings the z axis of joint 2 to lie
    // horizontally, and the arm then moves in a vertical plane
    const ChFrame<> X_12 =
        ChFrame<>(ChVector3d(0.0, 0.0, 0.0), rot_x(QUARTER_TURN)) * joint_rotation(q[1]);

    // The elbow is parallel to the shoulder, at the end of the upper arm
    const ChFrame<> X_23 = ChFrame<>(ChVector3d(arm.l1, 0.0, 0.0)) * joint_rotation(q[2]);

    // The end-effector sits at the end of the forearm
    const ChFrame<> X_3E = ChFrame<>(ChVector3d(arm.l2, 0.0, 0.0));

    const ChFrame<> X_W2 = X_W1 * X_12;
    const ChFrame<> X_W3 = X_W2 * X_23;
    return {X_W1, X_W2, X_W3, X_W3 * X_3E};
}

ChFrame<> spatial_fk(const SpatialArm &arm, const SpatialJoints &q)
{
    return spatial_fk_all(arm, q).back();
}

std::vector<SpatialJoints> spatial_ik(const SpatialArm &arm, const ChVector3d &p_W)
{
    const real_t r = std::hypot(p_W.x(), p_W.y());
    const real_t z = p_W.z() - arm.h0;
    const real_t facing = std::atan2(p_W.y(), p_W.x());

    std::vector<SpatialJoints> solutions;
    for (const auto &[q1, reach] : {std::pair{facing, r}, std::pair{wrap_angle(facing + PI), -r}})
    {
        for (const auto &planar : planar_ik({arm.l1, arm.l2}, reach, z))
        {
            solutions.push_back({q1, planar[0], planar[1]});
        }
    }
    return solutions;
}

std::optional<SpatialJoints> spatial_ik_elbow_up(const SpatialArm &arm, const ChVector3d &p_W)
{
    for (const auto &q : spatial_ik(arm, p_W))
    {
        // Facing the target means that the tip is in front of joint 1, and with the elbow up
        // the forearm bends down
        const bool facing = std::abs(wrap_angle(q[0] - std::atan2(p_W.y(), p_W.x()))) < 1.0e-9;
        if (facing && q[2] <= 0.0)
        {
            return q;
        }
    }
    return std::nullopt;
}

Eigen::Matrix3d numerical_jacobian(const SpatialArm &arm, const SpatialJoints &q)
{
    const real_t h = 1.0e-6;
    Eigen::Matrix3d J;
    for (int j = 0; j < 3; ++j)
    {
        SpatialJoints plus = q;
        SpatialJoints minus = q;
        plus[j] += h;
        minus[j] -= h;
        const ChVector3d column =
            (spatial_fk(arm, plus).GetPos() - spatial_fk(arm, minus).GetPos()) / (2.0 * h);
        J.col(j) << column.x(), column.y(), column.z();
    }
    return J;
}

IKResult numerical_ik(const SpatialArm &arm, const ChVector3d &target, SpatialJoints q0,
                      real_t damping, int max_iterations, real_t tolerance, real_t max_step,
                      const IKObserver &observer)
{
    IKResult result;
    result.q = q0;

    for (result.iterations = 0; result.iterations < max_iterations; ++result.iterations)
    {
        const ChVector3d e_W = target - spatial_fk(arm, result.q).GetPos();
        result.error = e_W.Length();

        if (observer)
        {
            observer(result.iterations, result.q, result.error);
        }

        if (result.error < tolerance)
        {
            break;
        }

        const Eigen::Matrix3d J = numerical_jacobian(arm, result.q);
        const Eigen::Vector3d e(e_W.x(), e_W.y(), e_W.z());

        Eigen::Vector3d dq;
        if (damping == 0.0)
        {
            dq = J.fullPivLu().solve(e);
        }
        else
        {
            const Eigen::Matrix3d A =
                J * J.transpose() + damping * damping * Eigen::Matrix3d::Identity();
            dq = J.transpose() * A.ldlt().solve(e);
        }

        // Near a singular J Newton's step can be huge. Keep its direction, limit its size
        if (dq.cwiseAbs().maxCoeff() > max_step)
        {
            dq *= max_step / dq.cwiseAbs().maxCoeff();
        }

        // The arm has stopped moving. When the target is out of reach this is the best it can do
        if (dq.norm() < tolerance)
        {
            break;
        }

        for (int j = 0; j < 3; ++j)
        {
            result.q[j] = wrap_angle(result.q[j] + dq(j));
        }
    }

    result.converged = result.error < tolerance;
    return result;
}

// ---------------------------------------------------------------------------
// Denavit-Hartenberg
// ---------------------------------------------------------------------------

ChFrame<> dh_frame(const DHRow &row)
{
    return joint_rotation(row.theta) * ChFrame<>(ChVector3d(row.a, 0.0, row.d), rot_x(row.alpha));
}

ChFrame<> dh_fk(const std::vector<DHRow> &table)
{
    ChFrame<> X;
    for (const auto &row : table)
    {
        X = X * dh_frame(row);
    }
    return X;
}

std::vector<DHRow> spatial_dh_table(const SpatialArm &arm, const SpatialJoints &q)
{
    return {{q[0], arm.h0, 0.0, QUARTER_TURN}, {q[1], 0.0, arm.l1, 0.0}, {q[2], 0.0, arm.l2, 0.0}};
}

} // namespace kinematics
} // namespace bitsim

#endif // BITSIM_CHRONO

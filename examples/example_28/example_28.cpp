#include "bitrl/bitrl_config.h"

#ifdef BITRL_CHRONO

#include "bitrl/bitrl_consts.h"
#include "bitrl/bitrl_types.h"

#include <chrono/core/ChFrame.h>
#include <chrono/core/ChMatrix33.h>
#include <chrono/functions/ChFunctionSetpoint.h>
#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChLinkMotorRotationAngle.h>
#include <chrono/physics/ChSystemNSC.h>

#ifdef BITRL_IRRLICHT
#include "bitrl/utils/render/irrlicht_utils.h"
#include <chrono/core/ChRealtimeStep.h>
#include <chrono_irrlicht/ChIrrTools.h>
#include <chrono_irrlicht/ChVisualSystemIrrlicht.h>
#endif

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace example_28
{

using bitrl::real_t;
using chrono::ChFrame;
using chrono::ChMatrix33d;
using chrono::ChVector3d;

// The same convention as in the previous lessons: X_AB is the pose of frame B
// expressed in frame A. W is the world frame, i the frame of joint i and E the
// end-effector frame. The arms are the ones of example_27

const real_t DEG = chrono::CH_DEG_TO_RAD;
const real_t PI = bitrl::consts::maths::PI;

const ChVector3d X_AXIS(1.0, 0.0, 0.0);
const ChVector3d Z_AXIS(0.0, 0.0, 1.0);

ChMatrix33d rot_x(real_t angle) { return ChMatrix33d(angle, X_AXIS); }
ChMatrix33d rot_z(real_t angle) { return ChMatrix33d(angle, Z_AXIS); }

ChFrame<> joint_rotation(real_t q) { return ChFrame<>(ChVector3d(0.0, 0.0, 0.0), rot_z(q)); }

real_t clean(real_t x) { return std::abs(x) < 5.0e-4 ? 0.0 : x; }

// Angles in (-180, 180] so that solutions are easy to compare
real_t wrap(real_t angle) { return std::atan2(std::sin(angle), std::cos(angle)); }

std::string to_string(const ChVector3d &v)
{
    std::ostringstream out;
    out << std::fixed << std::setprecision(3) << "(" << clean(v.x()) << ", " << clean(v.y()) << ", "
        << clean(v.z()) << ")";
    return out.str();
}

template <std::size_t N> std::string angles_to_string(const std::array<real_t, N> &q)
{
    std::ostringstream out;
    out << std::fixed << std::setprecision(1) << "(";
    for (std::size_t i = 0; i < N; ++i)
    {
        out << clean(q[i] / DEG) << (i + 1 < N ? ", " : ")");
    }
    return out.str();
}

void print_vector(const std::string &name, const ChVector3d &v)
{
    std::cout << "  " << std::left << std::setw(26) << name << " " << to_string(v) << "\n";
}

void print_flag(const std::string &name, bool value)
{
    std::cout << "  " << std::left << std::setw(26) << name << " " << std::boolalpha << value
              << "\n";
}

void print_error(const std::string &name, real_t value)
{
    std::cout << "  " << std::left << std::setw(26) << name << " " << std::scientific
              << std::setprecision(1) << value << "\n";
}

void print_title(const std::string &title) { std::cout << "\n" << title << "\n"; }

// ---------------------------------------------------------------------------
// The planar arm of example_27
// ---------------------------------------------------------------------------
struct PlanarArm
{
    real_t l1 = 0.4;
    real_t l2 = 0.3;
};

using PlanarJoints = std::array<real_t, 2>;

ChVector3d planar_fk(const PlanarArm &arm, const PlanarJoints &q)
{
    return {arm.l1 * std::cos(q[0]) + arm.l2 * std::cos(q[0] + q[1]),
            arm.l1 * std::sin(q[0]) + arm.l2 * std::sin(q[0] + q[1]), 0.0};
}

// Inverse kinematics of the planar arm. Returns no solution when the point is
// out of reach. Otherwise the two solutions, with the elbow on either side.
// They are the same when the arm is fully stretched or fully folded
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
        solutions.push_back({wrap(q1), q2});
    }

    if (std::abs(wrap(solutions[0][1] - solutions[1][1])) < 1.0e-6)
    {
        solutions.pop_back();
    }
    return solutions;
}

// d(x, y) / d(q1, q2)
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
// The spatial arm of example_27
// ---------------------------------------------------------------------------
struct SpatialArm
{
    real_t h0 = 0.3;
    real_t l1 = 0.4;
    real_t l2 = 0.3;
};

using Joints = std::array<real_t, 3>;

std::vector<ChFrame<>> spatial_fk_all(const SpatialArm &arm, const Joints &q)
{
    const ChFrame<> X_W1 = ChFrame<>(ChVector3d(0.0, 0.0, arm.h0)) * joint_rotation(q[0]);
    const ChFrame<> X_12 =
        ChFrame<>(ChVector3d(0.0, 0.0, 0.0), rot_x(90.0 * DEG)) * joint_rotation(q[1]);
    const ChFrame<> X_23 = ChFrame<>(ChVector3d(arm.l1, 0.0, 0.0)) * joint_rotation(q[2]);
    const ChFrame<> X_3E = ChFrame<>(ChVector3d(arm.l2, 0.0, 0.0));

    const ChFrame<> X_W2 = X_W1 * X_12;
    const ChFrame<> X_W3 = X_W2 * X_23;
    return {X_W1, X_W2, X_W3, X_W3 * X_3E};
}

ChVector3d spatial_fk(const SpatialArm &arm, const Joints &q)
{
    return spatial_fk_all(arm, q).back().GetPos();
}

// Inverse kinematics of the spatial arm. Joint 1 points the vertical plane of
// the arm at the target, and in that plane the shoulder and the elbow are the
// planar arm. The arm can also turn its back to the target and reach over its
// head, so there are up to four solutions
std::vector<Joints> spatial_ik(const SpatialArm &arm, const ChVector3d &p_W)
{
    const real_t r = std::hypot(p_W.x(), p_W.y());
    const real_t z = p_W.z() - arm.h0;
    const real_t facing = std::atan2(p_W.y(), p_W.x());

    std::vector<Joints> solutions;
    for (const auto &[q1, reach] : {std::pair{facing, r}, std::pair{wrap(facing + PI), -r}})
    {
        for (const auto &planar : planar_ik({arm.l1, arm.l2}, reach, z))
        {
            solutions.push_back({q1, planar[0], planar[1]});
        }
    }
    return solutions;
}

// The elbow-up solution facing the target, or nothing when it is out of reach
std::optional<Joints> spatial_ik_elbow_up(const SpatialArm &arm, const ChVector3d &p_W)
{
    for (const auto &q : spatial_ik(arm, p_W))
    {
        // Facing the target means that the tip is in front of joint 1 and with
        // the elbow up the forearm bends down
        const bool facing = std::abs(wrap(q[0] - std::atan2(p_W.y(), p_W.x()))) < 1.0e-9;
        if (facing && q[2] <= 0.0)
        {
            return q;
        }
    }
    return std::nullopt;
}

// d(x, y, z) / d(q1, q2, q3) by finite differences. This works for any arm as
// long as we have its forward kinematics
Eigen::Matrix3d numerical_jacobian(const SpatialArm &arm, const Joints &q)
{
    const real_t h = 1.0e-6;
    Eigen::Matrix3d J;
    for (int j = 0; j < 3; ++j)
    {
        Joints plus = q;
        Joints minus = q;
        plus[j] += h;
        minus[j] -= h;
        const ChVector3d column = (spatial_fk(arm, plus) - spatial_fk(arm, minus)) / (2.0 * h);
        J.col(j) << column.x(), column.y(), column.z();
    }
    return J;
}

struct IKResult
{
    Joints q;
    int iterations;
    real_t error;
};

// Iterate q <- q + dq until the tip reaches the target. With damping = 0 the
// step is Newton's method, dq = J^-1 e. Otherwise it is damped least squares,
// dq = J^T (J J^T + damping^2 I)^-1 e
IKResult numerical_ik(const SpatialArm &arm, const ChVector3d &target, Joints q, real_t damping,
                      bool verbose, int max_iterations = 100)
{
    const real_t tolerance = 1.0e-6;
    const real_t max_step = 0.5; // never turn a joint by more than this in one step

    real_t error = 0.0;
    int iteration = 0;
    for (; iteration < max_iterations; ++iteration)
    {
        const ChVector3d e_W = target - spatial_fk(arm, q);
        error = e_W.Length();

        // The first iterations, then every fifth, then the last
        if (verbose && (iteration < 8 || iteration % 5 == 0 || error < tolerance))
        {
            std::cout << "  " << std::right << std::setw(9) << iteration << "   " << std::left
                      << std::setw(22) << angles_to_string(q) << std::scientific
                      << std::setprecision(1) << error << "\n";
        }

        if (error < tolerance)
        {
            break;
        }

        const Eigen::Matrix3d J = numerical_jacobian(arm, q);
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

        // The arm has stopped moving. When the target is out of reach this is
        // the best we can do
        if (dq.norm() < tolerance)
        {
            break;
        }

        for (int j = 0; j < 3; ++j)
        {
            q[j] = wrap(q[j] + dq(j));
        }
    }

    return {q, iteration, error};
}

// ---------------------------------------------------------------------------
// The sections of the lesson
// ---------------------------------------------------------------------------
void planar_solutions(const PlanarArm &arm)
{
    print_title("1. Inverse kinematics of the planar arm");

    for (const ChVector3d &target : {ChVector3d(0.4, 0.3, 0.0), ChVector3d(-0.2, 0.5, 0.0)})
    {
        print_vector("target", target);
        for (const auto &q : planar_ik(arm, target.x(), target.y()))
        {
            print_vector("  q = " + angles_to_string(q) + " gives", planar_fk(arm, q));
        }
    }
}

void reachability(const PlanarArm &arm)
{
    print_title("2. How many solutions");

    std::cout << "  target                     distance  solutions\n";
    for (const ChVector3d &target :
         {ChVector3d(0.8, 0.0, 0.0), ChVector3d(0.7, 0.0, 0.0), ChVector3d(0.5, 0.0, 0.0),
          ChVector3d(0.1, 0.0, 0.0), ChVector3d(0.05, 0.0, 0.0)})
    {
        const auto solutions = planar_ik(arm, target.x(), target.y());
        std::cout << "  " << std::left << std::setw(26) << to_string(target) << " " << std::fixed
                  << std::setprecision(3) << std::setw(9) << target.Length() << " "
                  << solutions.size() << "\n";
    }
}

void spatial_solutions(const SpatialArm &arm)
{
    print_title("3. Inverse kinematics of the spatial arm");

    // The end-effector position of section 4 in example_27
    const ChVector3d target = spatial_fk(arm, {90.0 * DEG, 30.0 * DEG, -60.0 * DEG});
    print_vector("target", target);

    for (const auto &q : spatial_ik(arm, target))
    {
        const ChVector3d reached = spatial_fk(arm, q);
        std::cout << "    q = " << std::left << std::setw(22) << angles_to_string(q) << " gives "
                  << to_string(reached) << "\n";
    }
}

void jacobian(const PlanarArm &arm)
{
    print_title("4. The Jacobian");

    // The Jacobian maps joint speeds to the speed of the tip
    const PlanarJoints q{30.0 * DEG, 60.0 * DEG};
    const Eigen::Matrix2d J = planar_jacobian(arm, q);

    // Compare with finite differences of the forward kinematics
    const real_t h = 1.0e-6;
    Eigen::Matrix2d J_fd;
    for (int j = 0; j < 2; ++j)
    {
        PlanarJoints plus = q;
        PlanarJoints minus = q;
        plus[j] += h;
        minus[j] -= h;
        const ChVector3d column = (planar_fk(arm, plus) - planar_fk(arm, minus)) / (2.0 * h);
        J_fd.col(j) << column.x(), column.y();
    }

    std::cout << "  J at (30, 60)\n" << std::fixed << std::setprecision(3) << std::right;
    for (int row = 0; row < 2; ++row)
    {
        std::cout << "    " << std::setw(7) << J(row, 0) << " " << std::setw(7) << J(row, 1)
                  << "\n";
    }
    print_error("difference to numerical J", (J - J_fd).norm());

    // How fast must the joints turn to move the tip at 0.1 m/s away from the
    // shoulder? Along the arm, as it gets straighter
    std::cout << "\n      q2   det(J)   q1 dot   q2 dot  (rad/s)\n";
    for (const real_t q2 : {90.0, 30.0, 10.0, 2.0, 0.5})
    {
        const PlanarJoints joints{0.0, q2 * DEG};
        const Eigen::Matrix2d Jq = planar_jacobian(arm, joints);
        const ChVector3d tip = planar_fk(arm, joints);
        const ChVector3d away = tip.GetNormalized() * 0.1;
        const Eigen::Vector2d q_dot = Jq.fullPivLu().solve(Eigen::Vector2d(away.x(), away.y()));

        std::cout << "  " << std::fixed << std::right << std::setprecision(1) << std::setw(6) << q2
                  << std::setprecision(4) << std::setw(9) << Jq.determinant()
                  << std::setprecision(2) << std::setw(9) << q_dot(0) << std::setw(9) << q_dot(1)
                  << "\n";
    }
}

void numerical_solution(const SpatialArm &arm)
{
    print_title("5. Numerical inverse kinematics");

    const ChVector3d target = spatial_fk(arm, {90.0 * DEG, 30.0 * DEG, -60.0 * DEG});
    const Joints start{0.0, 45.0 * DEG, -45.0 * DEG};

    std::cout << "  Newton's method\n  iteration   q                      error\n";
    const IKResult newton = numerical_ik(arm, target, start, 0.0, true);
    print_vector("reached", spatial_fk(arm, newton.q));

    std::cout << "\n  Damped least squares\n  iteration   q                      error\n";
    const IKResult dls = numerical_ik(arm, target, start, 0.2, true);
    print_vector("reached", spatial_fk(arm, dls.q));

    // A target 1 m away from the shoulder is out of reach. The best the arm can
    // do is to stretch towards it
    const ChVector3d far_target(0.8, 0.0, 0.9);
    std::cout << "\n";
    print_vector("out of reach target", far_target);
    for (const auto &[name, damping] : {std::pair{"Newton", 0.0}, std::pair{"damped", 0.2}})
    {
        const IKResult result = numerical_ik(arm, far_target, start, damping, false);
        std::cout << "  " << std::left << std::setw(8) << name << "after " << std::setw(4)
                  << result.iterations << "iterations q = " << std::setw(22)
                  << angles_to_string(result.q) << "error " << std::fixed << std::setprecision(3)
                  << result.error << "\n";
    }
    std::cout << "  " << std::left << std::setw(26) << "closest possible error"
              << " " << std::fixed << std::setprecision(3)
              << (far_target - ChVector3d(0.0, 0.0, arm.h0)).Length() - arm.l1 - arm.l2 << "\n";
}

std::shared_ptr<chrono::ChBody> add_link(chrono::ChSystemNSC &system, const ChFrame<> &X_Wi,
                                         real_t length)
{
    auto link = chrono_types::make_shared<chrono::ChBodyEasyBox>(std::max(length, 0.05), 0.05, 0.05,
                                                                 1000.0, true, false);
    link->SetCoordsys((X_Wi * ChFrame<>(ChVector3d(length / 2.0, 0.0, 0.0))).GetCoordsys());
    system.Add(link);
    return link;
}

std::shared_ptr<chrono::ChLinkMotorRotationAngle>
add_joint(chrono::ChSystemNSC &system, std::shared_ptr<chrono::ChBody> child,
          std::shared_ptr<chrono::ChBody> parent, const ChFrame<> &X_Wi,
          std::shared_ptr<chrono::ChFunction> angle)
{
    auto motor = chrono_types::make_shared<chrono::ChLinkMotorRotationAngle>();
    motor->Initialize(child, parent, X_Wi);
    motor->SetAngleFunction(angle);
    system.Add(motor);
    return motor;
}

// The Chrono model of example_27. Here we tell each motor where to be at every step
struct ChronoArm
{
    std::shared_ptr<chrono::ChBody> base;
    std::shared_ptr<chrono::ChBody> turret;
    std::shared_ptr<chrono::ChBody> upper_arm;
    std::shared_ptr<chrono::ChBody> forearm;
    std::array<std::shared_ptr<chrono::ChFunctionSetpoint>, 3> setpoints;
    real_t l2;

    // The joints have to be at q at time t
    void command(const Joints &q, real_t t) const
    {
        for (int j = 0; j < 3; ++j)
        {
            setpoints[j]->SetSetpoint(q[j], t);
        }
    }

    ChVector3d tip() const
    {
        return forearm->GetFrameRefToAbs().TransformPointLocalToParent(
            ChVector3d(l2 / 2.0, 0.0, 0.0));
    }
};

// Builds the arm at q = 0 and then moves it to q_start
ChronoArm build_chrono_arm(chrono::ChSystemNSC &system, const SpatialArm &arm,
                           const Joints &q_start)
{
    system.SetGravitationalAcceleration(ChVector3d(0.0, 0.0, -9.81));
    system.SetSolverType(chrono::ChSolver::Type::SPARSE_QR);

    const auto frames = spatial_fk_all(arm, {0.0, 0.0, 0.0});

    auto base =
        chrono_types::make_shared<chrono::ChBodyEasyBox>(0.2, 0.2, arm.h0, 1000.0, true, false);
    base->SetPos(ChVector3d(0.0, 0.0, arm.h0 / 2.0));
    base->SetFixed(true);
    system.Add(base);

    auto turret = add_link(system, frames[0], 0.0);
    auto upper_arm = add_link(system, frames[1], arm.l1);
    auto forearm = add_link(system, frames[2], arm.l2);

    std::array<std::shared_ptr<chrono::ChFunctionSetpoint>, 3> setpoints;
    for (auto &setpoint : setpoints)
    {
        setpoint = chrono_types::make_shared<chrono::ChFunctionSetpoint>();
    }
    add_joint(system, turret, base, frames[0], setpoints[0]);
    add_joint(system, upper_arm, turret, frames[1], setpoints[1]);
    add_joint(system, forearm, upper_arm, frames[2], setpoints[2]);

    // Move the arm to the start before the clock starts
    const ChronoArm model{base, turret, upper_arm, forearm, setpoints, arm.l2};
    model.command(q_start, 0.0);
    system.DoAssembly(chrono::AssemblyAnalysis::Level::POSITION);
    return model;
}

// The tip has to move from A to B along a straight line in 2 s. On the way
// joint 1 has to turn by 90 degrees
const ChVector3d LINE_START(0.5, -0.2, 0.2);
const ChVector3d LINE_END(0.2, 0.5, 0.5);
const real_t LINE_DURATION = 2.0;

ChVector3d point_on_line(real_t t)
{
    // smooth start and stop: s goes from 0 to 1 with zero speed at both ends
    const real_t s = 0.5 - 0.5 * std::cos(PI * std::clamp(t / LINE_DURATION, 0.0, 1.0));
    return LINE_START + (LINE_END - LINE_START) * s;
}

real_t distance_off_line(const ChVector3d &p)
{
    const ChVector3d along = (LINE_END - LINE_START).GetNormalized();
    const ChVector3d from_start = p - LINE_START;
    return (from_start - along * from_start.Dot(along)).Length();
}

void follow_a_line(const SpatialArm &arm)
{
    print_title("6. Following a straight line in simulation");

    chrono::ChSystemNSC system;
    const ChronoArm model = build_chrono_arm(system, arm, *spatial_ik_elbow_up(arm, LINE_START));

    std::cout << "     t   q                      target                   tip in simulation"
                 "        off the line\n";

    const real_t step_size = 0.001;
    real_t max_distance = 0.0;
    int step = 0;
    while (system.GetChTime() < LINE_DURATION - step_size / 2.0)
    {
        const real_t t_next = system.GetChTime() + step_size;
        const ChVector3d target = point_on_line(t_next);
        const Joints q = *spatial_ik_elbow_up(arm, target);
        model.command(q, t_next);

        system.DoStepDynamics(step_size);
        ++step;

        // How far is the simulated tip from the line through A and B?
        const ChVector3d tip = model.tip();
        const real_t distance = distance_off_line(tip);
        max_distance = std::max(max_distance, distance);

        if (step % 400 == 0)
        {
            std::cout << "  " << std::fixed << std::setprecision(2) << std::right << std::setw(4)
                      << system.GetChTime() << "   " << std::left << std::setw(22)
                      << angles_to_string(q) << " " << std::setw(24) << to_string(target) << " "
                      << std::setw(25) << to_string(tip) << std::scientific << std::setprecision(1)
                      << distance << "\n";
        }
    }

    print_error("largest distance off line", max_distance);
}

#ifdef BITRL_IRRLICHT
// The arm of section 6 goes along the line and back, again and again. The
// other solutions for the same target are drawn as thin grey arms
void render(const SpatialArm &arm)
{
    using namespace bitrl::utils::render;

    chrono::ChSystemNSC system;
    const ChronoArm model = build_chrono_arm(system, arm, *spatial_ik_elbow_up(arm, LINE_START));
    model.base->GetVisualShape(0)->SetColor(chrono::ChColor(0.30f, 0.30f, 0.30f));
    model.turret->GetVisualShape(0)->SetColor(chrono::ChColor(0.30f, 0.30f, 0.30f));
    model.upper_arm->GetVisualShape(0)->SetColor(chrono::ChColor(0.55f, 0.70f, 0.90f));
    model.forearm->GetVisualShape(0)->SetColor(chrono::ChColor(0.90f, 0.60f, 0.30f));

    chrono::irrlicht::ChVisualSystemIrrlicht vis;
    vis.AttachSystem(&system);
    vis.SetCameraVertical(chrono::CameraVerticalDir::Z);
    vis.SetWindowSize(1280, 720);
    vis.SetWindowTitle("Example 28: Inverse kinematics");
    vis.Initialize();
    vis.AddCamera({1.4, -1.2, 1.0}, {0.1, 0.15, 0.3});
    vis.GetActiveCamera()->setFOV(0.75f);
    vis.AddTypicalLights();
    vis.BindAll();

    auto *readout = vis.GetGUIEnvironment()->addStaticText(
        L"", irr::core::rect<irr::s32>(1020, 10, 1270, 140), true, true);
    readout->setBackgroundColor(irr::video::SColor(220, 255, 255, 255));
    readout->setDrawBackground(true);

    // A to B, wait, B to A, wait
    const real_t wait = 1.0;
    const real_t cycle = 2.0 * (LINE_DURATION + wait);
    const auto target_at = [&](real_t t)
    {
        const real_t phase = std::fmod(t, cycle);
        const real_t back = cycle / 2.0;
        return phase < back ? point_on_line(phase) : point_on_line(LINE_DURATION - (phase - back));
    };

    std::vector<ChVector3d> trail;
    const std::size_t trail_length = 300;

    const real_t step_size = 0.001;
    const int steps_per_frame = 10;
    chrono::ChRealtimeStepTimer real_time;

    Joints q = *spatial_ik_elbow_up(arm, LINE_START);
    while (vis.Run())
    {
        const ChVector3d target = target_at(system.GetChTime());
        const ChVector3d tip = model.tip();
        const auto solutions = spatial_ik(arm, target);

        trail.push_back(tip);
        if (trail.size() > trail_length)
        {
            trail.erase(trail.begin());
        }

        std::ostringstream text;
        text << "t = " << std::fixed << std::setprecision(1) << system.GetChTime() << " s\n\n"
             << "target:  " << to_string(target) << "\n"
             << "q = " << angles_to_string(q) << " deg\n"
             << "tip in Chrono:  " << to_string(tip) << "\n"
             << "off the line = " << std::scientific << std::setprecision(1)
             << distance_off_line(tip) << " m\n\n"
             << solutions.size() << " solutions, elbow up in colour\n";
        readout->setText(irr::core::stringw(text.str().c_str()).c_str());

        vis.BeginScene();
        vis.Render();

        chrono::irrlicht::tools::drawGrid(&vis, 0.25, 0.25, 8, 8, chrono::ChCoordsysd(),
                                          chrono::ChColor(0.35f, 0.45f, 0.55f));
        irr_draw_frame(vis, ChFrame<>(), 0.3, "W");

        irr_draw_line(vis, LINE_START, LINE_END, chrono::ChColor(1.0f, 0.8f, 0.2f), 1.0);
        irr_draw_label(vis, "A", LINE_START);
        irr_draw_label(vis, "B", LINE_END);
        for (std::size_t i = 1; i < trail.size(); ++i)
        {
            irr_draw_line(vis, trail[i - 1], trail[i], chrono::ChColor(1.0f, 0.4f, 0.1f), 3.0);
        }

        // The other solutions: shoulder to elbow to tip
        for (const auto &other : solutions)
        {
            if (std::abs(wrap(other[0] - q[0])) + std::abs(other[1] - q[1]) +
                    std::abs(other[2] - q[2]) <
                1.0e-6)
            {
                continue;
            }
            const auto frames = spatial_fk_all(arm, other);
            irr_draw_line(vis, frames[1].GetPos(), frames[2].GetPos(),
                          chrono::ChColor(0.7f, 0.7f, 0.7f), 2.0);
            irr_draw_line(vis, frames[2].GetPos(), frames[3].GetPos(),
                          chrono::ChColor(0.7f, 0.7f, 0.7f), 2.0);
        }

        for (int i = 0; i < steps_per_frame; ++i)
        {
            const real_t t_next = system.GetChTime() + step_size;
            q = *spatial_ik_elbow_up(arm, target_at(t_next));
            model.command(q, t_next);
            system.DoStepDynamics(step_size);
        }
        vis.EndScene();
        real_time.Spin(steps_per_frame * step_size);
    }
}
#endif

} // namespace example_28

int main(int argc, char **argv)
{
    using namespace example_28;

    const PlanarArm planar;
    const SpatialArm spatial;

    planar_solutions(planar);
    reachability(planar);
    spatial_solutions(spatial);
    jacobian(planar);
    numerical_solution(spatial);
    follow_a_line(spatial);

    if (argc > 1 && std::string(argv[1]) == "--render")
    {
#ifdef BITRL_IRRLICHT
        render(spatial);
#else
        std::cerr << "--render needs bitrl configured with BITRL_IRRLICHT=ON" << std::endl;
        return 1;
#endif
    }

    return 0;
}

#else
#include <iostream>
int main()
{
    std::cerr << "You need PROJECTCHRONO configured with "
              << "bitrl in order to run this example "
              << "Reconfigure bitrl and set ENABLE_CHRONO=ON" << std::endl;
    return 1;
}
#endif

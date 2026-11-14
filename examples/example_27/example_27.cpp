#include "bitrl/bitrl_config.h"

#ifdef BITRL_CHRONO

#include "bitrl/bitrl_consts.h"
#include "bitrl/bitrl_types.h"

#include <chrono/core/ChFrame.h>
#include <chrono/core/ChMatrix33.h>
#include <chrono/functions/ChFunctionSine.h>
#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChLinkMotorRotationAngle.h>
#include <chrono/physics/ChSystemNSC.h>

#ifdef BITRL_IRRLICHT
#include "bitrl/utils/render/irrlicht_utils.h"
#include <chrono/core/ChRealtimeStep.h>
#include <chrono_irrlicht/ChIrrTools.h>
#include <chrono_irrlicht/ChVisualSystemIrrlicht.h>
#endif

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace example_27
{

using bitrl::real_t;
using chrono::ChFrame;
using chrono::ChMatrix33d;
using chrono::ChVector3d;

// The same convention as in the previous lessons: X_AB is the pose of frame B
// expressed in frame A, so that X_AB * X_BC = X_AC and X_AB * p_B = p_A.
// W is the world frame, i the frame of joint i and E the end-effector frame

const real_t DEG = chrono::CH_DEG_TO_RAD;

const ChVector3d X_AXIS(1.0, 0.0, 0.0);
const ChVector3d Z_AXIS(0.0, 0.0, 1.0);

// The elementary rotations of example_26
ChMatrix33d rot_x(real_t angle) { return ChMatrix33d(angle, X_AXIS); }
ChMatrix33d rot_z(real_t angle) { return ChMatrix33d(angle, Z_AXIS); }

// Every joint of this lesson is revolute and turns around its own z axis.
// Chrono motors use the same convention
ChFrame<> joint_rotation(real_t q) { return ChFrame<>(ChVector3d(0.0, 0.0, 0.0), rot_z(q)); }

real_t clean(real_t x) { return std::abs(x) < 5.0e-4 ? 0.0 : x; }

std::string to_string(const ChVector3d &v)
{
    std::ostringstream out;
    out << std::fixed << std::setprecision(3) << "(" << clean(v.x()) << ", " << clean(v.y()) << ", "
        << clean(v.z()) << ")";
    return out.str();
}

void print_matrix(const std::string &name, const ChMatrix33d &R)
{
    std::cout << "  " << name << "\n";
    for (int row = 0; row < 3; ++row)
    {
        std::cout << "    " << std::fixed << std::setprecision(3);
        for (int col = 0; col < 3; ++col)
        {
            std::cout << std::right << std::setw(7) << clean(R(row, col)) << " ";
        }
        std::cout << "\n";
    }
}

void print_vector(const std::string &name, const ChVector3d &v)
{
    std::cout << "  " << std::left << std::setw(26) << name << " " << to_string(v) << "\n";
}

void print_number(const std::string &name, real_t value)
{
    std::cout << "  " << std::left << std::setw(26) << name << " " << std::fixed
              << std::setprecision(3) << value << "\n";
}

void print_flag(const std::string &name, bool value)
{
    std::cout << "  " << std::left << std::setw(26) << name << " " << std::boolalpha << value
              << "\n";
}

void print_title(const std::string &title) { std::cout << "\n" << title << "\n"; }

bool same_point(const ChVector3d &a, const ChVector3d &b, real_t tolerance = 1.0e-9)
{
    return (a - b).Length() < tolerance;
}

// ---------------------------------------------------------------------------
// A planar arm with two links that moves in the xy plane, like a SCARA robot
// seen from above. Joint 1 is at the origin of the world
// ---------------------------------------------------------------------------
struct PlanarArm
{
    real_t l1 = 0.4;
    real_t l2 = 0.3;
};

// The arm as a chain of frames. Each link contributes a fixed offset and each
// joint a rotation around z
ChFrame<> planar_fk_frames(const PlanarArm &arm, real_t q1, real_t q2)
{
    const ChFrame<> X_W1 = joint_rotation(q1);
    const ChFrame<> X_12 = ChFrame<>(ChVector3d(arm.l1, 0.0, 0.0)) * joint_rotation(q2);
    const ChFrame<> X_2E = ChFrame<>(ChVector3d(arm.l2, 0.0, 0.0));
    return X_W1 * X_12 * X_2E;
}

// The same arm written out with trigonometry
ChVector3d planar_fk_formula(const PlanarArm &arm, real_t q1, real_t q2)
{
    return {arm.l1 * std::cos(q1) + arm.l2 * std::cos(q1 + q2),
            arm.l1 * std::sin(q1) + arm.l2 * std::sin(q1 + q2), 0.0};
}

// ---------------------------------------------------------------------------
// A spatial arm with three joints. Joint 1 turns the arm around the vertical
// axis, joints 2 and 3 are the shoulder and the elbow
// ---------------------------------------------------------------------------
struct SpatialArm
{
    real_t h0 = 0.3; // height of the shoulder above the floor
    real_t l1 = 0.4; // upper arm
    real_t l2 = 0.3; // forearm
};

using Joints = std::array<real_t, 3>;

// The pose of every joint frame in the world, followed by the end-effector.
// The Chrono model in section 6 uses these frames to place its bodies
std::vector<ChFrame<>> spatial_fk_all(const SpatialArm &arm, const Joints &q)
{
    // Joint 1 sits on the base and turns around the vertical axis
    const ChFrame<> X_W1 = ChFrame<>(ChVector3d(0.0, 0.0, arm.h0)) * joint_rotation(q[0]);

    // The shoulder turns around a horizontal axis. Rx(90) brings the z axis of
    // joint 2 to lie horizontally, and the arm then moves in a vertical plane
    const ChFrame<> X_12 =
        ChFrame<>(ChVector3d(0.0, 0.0, 0.0), rot_x(90.0 * DEG)) * joint_rotation(q[1]);

    // The elbow is parallel to the shoulder, at the end of the upper arm
    const ChFrame<> X_23 = ChFrame<>(ChVector3d(arm.l1, 0.0, 0.0)) * joint_rotation(q[2]);

    // The end-effector sits at the end of the forearm
    const ChFrame<> X_3E = ChFrame<>(ChVector3d(arm.l2, 0.0, 0.0));

    const ChFrame<> X_W2 = X_W1 * X_12;
    const ChFrame<> X_W3 = X_W2 * X_23;
    const ChFrame<> X_WE = X_W3 * X_3E;
    return {X_W1, X_W2, X_W3, X_WE};
}

ChFrame<> spatial_fk(const SpatialArm &arm, const Joints &q)
{
    return spatial_fk_all(arm, q).back();
}

// ---------------------------------------------------------------------------
// Denavit-Hartenberg parameters
// ---------------------------------------------------------------------------
struct DHRow
{
    real_t theta; // rotation around z
    real_t d;     // offset along z
    real_t a;     // offset along the new x
    real_t alpha; // rotation around the new x
};

// Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
ChFrame<> dh_frame(const DHRow &row)
{
    return joint_rotation(row.theta) * ChFrame<>(ChVector3d(row.a, 0.0, row.d), rot_x(row.alpha));
}

std::vector<DHRow> spatial_dh_table(const SpatialArm &arm, const Joints &q)
{
    return {{q[0], arm.h0, 0.0, 90.0 * DEG}, {q[1], 0.0, arm.l1, 0.0}, {q[2], 0.0, arm.l2, 0.0}};
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

// ---------------------------------------------------------------------------
// The sections of the lesson
// ---------------------------------------------------------------------------
void one_joint()
{
    print_title("1. A joint is a frame that moves");

    // A link of length 0.4 on a joint at (1, 0, 0). The frame of the joint
    // has a fixed part, where the joint is mounted, and a part that depends on q
    const ChFrame<> mount(ChVector3d(1.0, 0.0, 0.0));
    const ChVector3d tip_in_link(0.4, 0.0, 0.0);

    for (const real_t q : {0.0, 45.0, 90.0, 180.0})
    {
        const ChFrame<> X_W1 = mount * joint_rotation(q * DEG);
        std::ostringstream name;
        name << "tip at q = " << q;
        print_vector(name.str(), X_W1 * tip_in_link);
    }

    // The tip moves on a circle of radius 0.4 around the joint
    const ChFrame<> X_W1 = mount * joint_rotation(123.0 * DEG);
    print_number("distance tip-joint", (X_W1 * tip_in_link - mount.GetPos()).Length());
}

void planar_arm(const PlanarArm &arm)
{
    print_title("2. A planar arm with two links");

    std::cout
        << "      q1      q2   chain of frames             formula                     same\n";
    for (const auto &[q1, q2] : std::vector<std::array<real_t, 2>>{
             {0.0, 0.0}, {90.0, 0.0}, {0.0, 90.0}, {30.0, 60.0}, {45.0, -90.0}, {120.0, 150.0}})
    {
        const ChVector3d chain = planar_fk_frames(arm, q1 * DEG, q2 * DEG).GetPos();
        const ChVector3d formula = planar_fk_formula(arm, q1 * DEG, q2 * DEG);
        std::cout << "  " << std::fixed << std::setprecision(1) << std::right << std::setw(6) << q1
                  << "  " << std::setw(6) << q2 << "   " << std::left << std::setw(28)
                  << to_string(chain) << std::setw(28) << to_string(formula) << std::boolalpha
                  << same_point(chain, formula) << "\n";
    }

    // The orientation of the end-effector is the sum of the joint angles
    const ChFrame<> X_WE = planar_fk_frames(arm, 30.0 * DEG, 60.0 * DEG);
    print_vector("tool x axis at (30, 60)", X_WE.GetRotMat().GetAxisX());
}

void workspace(const PlanarArm &arm)
{
    print_title("3. The workspace");

    // Try every pair of joint angles on a grid and record how far the tip gets
    real_t closest = 1.0e9;
    real_t furthest = 0.0;
    const int n = 360;
    for (int i = 0; i < n; ++i)
    {
        for (int j = 0; j < n; ++j)
        {
            const real_t q1 = 2.0 * bitrl::consts::maths::PI * i / n;
            const real_t q2 = 2.0 * bitrl::consts::maths::PI * j / n;
            const real_t r = planar_fk_formula(arm, q1, q2).Length();
            closest = std::min(closest, r);
            furthest = std::max(furthest, r);
        }
    }

    print_number("closest reach on the grid", closest);
    print_number("|l1 - l2|", std::abs(arm.l1 - arm.l2));
    print_number("furthest reach on the grid", furthest);
    print_number("l1 + l2", arm.l1 + arm.l2);

    // Many joint angles, one tip position: (q1, q2) and its mirror image
    const ChVector3d up = planar_fk_formula(arm, 0.0, 90.0 * DEG);
    const real_t q2 = -90.0 * DEG;
    const real_t q1 = 2.0 * std::atan2(arm.l2, arm.l1);
    const ChVector3d down = planar_fk_formula(arm, q1, q2);
    print_vector("tip with elbow on one side", up);
    print_vector("tip with elbow on the other", down);
}

void spatial_arm(const SpatialArm &arm)
{
    print_title("4. A spatial arm with three joints");

    const Joints q{90.0 * DEG, 30.0 * DEG, -60.0 * DEG};
    const auto frames = spatial_fk_all(arm, q);

    print_vector("joint 1", frames[0].GetPos());
    print_vector("joint 2 (shoulder)", frames[1].GetPos());
    print_vector("joint 3 (elbow)", frames[2].GetPos());
    print_vector("end-effector", frames[3].GetPos());
    print_matrix("R_WE", frames[3].GetRotMat());

    // The same result from the geometry: the reach in the vertical plane of the
    // arm and the height, followed by the turn of joint 1
    const real_t r = arm.l1 * std::cos(q[1]) + arm.l2 * std::cos(q[1] + q[2]);
    const real_t z = arm.h0 + arm.l1 * std::sin(q[1]) + arm.l2 * std::sin(q[1] + q[2]);
    const ChVector3d formula(r * std::cos(q[0]), r * std::sin(q[0]), z);
    print_vector("from the formula", formula);
    print_flag("same", same_point(frames[3].GetPos(), formula));
}

void denavit_hartenberg(const SpatialArm &arm)
{
    print_title("5. Denavit-Hartenberg parameters");

    const Joints q{90.0 * DEG, 30.0 * DEG, -60.0 * DEG};
    const auto table = spatial_dh_table(arm, q);

    std::cout << "  joint   theta       d       a   alpha\n";
    for (std::size_t i = 0; i < table.size(); ++i)
    {
        std::cout << "  " << std::right << std::setw(5) << i + 1 << std::fixed
                  << std::setprecision(3) << std::setw(8) << table[i].theta / DEG << std::setw(8)
                  << table[i].d << std::setw(8) << table[i].a << std::setw(8)
                  << table[i].alpha / DEG << "\n";
    }

    const ChFrame<> X_WE_dh = dh_fk(table);
    const ChFrame<> X_WE = spatial_fk(arm, q);
    print_vector("end-effector from DH", X_WE_dh.GetPos());
    print_flag("same position", same_point(X_WE_dh.GetPos(), X_WE.GetPos()));
    print_flag("same orientation", (X_WE_dh.GetRotMat() - X_WE.GetRotMat()).norm() < 1.0e-9);
}

// A link is a thin box along the x axis of its joint frame, with its centre
// half way along
std::shared_ptr<chrono::ChBody> add_link(chrono::ChSystemNSC &system, const ChFrame<> &X_Wi,
                                         real_t length)
{
    auto link = chrono_types::make_shared<chrono::ChBodyEasyBox>(std::max(length, 0.05), 0.05, 0.05,
                                                                 1000.0, true, false);
    link->SetCoordsys((X_Wi * ChFrame<>(ChVector3d(length / 2.0, 0.0, 0.0))).GetCoordsys());
    system.Add(link);
    return link;
}

// The joint turns child with respect to parent around the z axis of X_Wi
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

// The Chrono model of the spatial arm
struct ChronoArm
{
    std::shared_ptr<chrono::ChBody> base;
    std::shared_ptr<chrono::ChBody> turret;
    std::shared_ptr<chrono::ChBody> upper_arm;
    std::shared_ptr<chrono::ChBody> forearm;
    std::array<std::shared_ptr<chrono::ChLinkMotorRotationAngle>, 3> motors;
    real_t l2;

    Joints joints() const
    {
        return {motors[0]->GetMotorAngle(), motors[1]->GetMotorAngle(), motors[2]->GetMotorAngle()};
    }

    // The tip of the forearm, half a link from its centre
    ChVector3d tip() const
    {
        return forearm->GetFrameRefToAbs().TransformPointLocalToParent(
            ChVector3d(l2 / 2.0, 0.0, 0.0));
    }
};

// Each joint follows its own sine wave. The motors know nothing about our FK
ChronoArm build_chrono_arm(chrono::ChSystemNSC &system, const SpatialArm &arm)
{
    system.SetGravitationalAcceleration(ChVector3d(0.0, 0.0, -9.81));

    // The default iterative solver lets the joints give a little under load.
    // A direct solver keeps them stiff, so that the bodies follow the motors exactly
    system.SetSolverType(chrono::ChSolver::Type::SPARSE_QR);

    // Build the arm in its zero configuration, using the joint frames of the FK
    const auto frames = spatial_fk_all(arm, {0.0, 0.0, 0.0});

    auto base =
        chrono_types::make_shared<chrono::ChBodyEasyBox>(0.2, 0.2, arm.h0, 1000.0, true, false);
    base->SetPos(ChVector3d(0.0, 0.0, arm.h0 / 2.0));
    base->SetFixed(true);
    system.Add(base);

    // The turret only turns, it has no length
    auto turret = add_link(system, frames[0], 0.0);
    auto upper_arm = add_link(system, frames[1], arm.l1);
    auto forearm = add_link(system, frames[2], arm.l2);

    std::array<std::shared_ptr<chrono::ChLinkMotorRotationAngle>, 3> motors{
        add_joint(system, turret, base, frames[0],
                  chrono_types::make_shared<chrono::ChFunctionSine>(90.0 * DEG, 0.25)),
        add_joint(system, upper_arm, turret, frames[1],
                  chrono_types::make_shared<chrono::ChFunctionSine>(45.0 * DEG, 0.5)),
        add_joint(system, forearm, upper_arm, frames[2],
                  chrono_types::make_shared<chrono::ChFunctionSine>(-60.0 * DEG, 0.4))};

    return {base, turret, upper_arm, forearm, motors, arm.l2};
}

void check_in_simulation(const SpatialArm &arm)
{
    print_title("6. Checking against a simulation");

    chrono::ChSystemNSC system;
    const ChronoArm model = build_chrono_arm(system, arm);

    std::cout << "     t      q1      q2      q3   tip in simulation        tip from FK"
                 "              error\n";

    const real_t step_size = 0.001;
    real_t max_error = 0.0;
    int step = 0;
    while (system.GetChTime() < 2.0)
    {
        system.DoStepDynamics(step_size);
        ++step;

        const Joints q = model.joints();
        const ChVector3d simulated = model.tip();
        const ChVector3d predicted = spatial_fk(arm, q).GetPos();
        const real_t error = (simulated - predicted).Length();
        max_error = std::max(max_error, error);

        if (step % 400 == 0)
        {
            std::cout << "  " << std::fixed << std::setprecision(2) << std::right << std::setw(4)
                      << system.GetChTime() << std::setprecision(1) << std::setw(8) << q[0] / DEG
                      << std::setw(8) << q[1] / DEG << std::setw(8) << q[2] / DEG << "   "
                      << std::left << std::setw(25) << to_string(simulated) << std::setw(25)
                      << to_string(predicted) << std::scientific << std::setprecision(1) << error
                      << "\n";
        }
    }

    std::cout << "  " << std::left << std::setw(26) << "largest error"
              << " " << std::scientific << std::setprecision(1) << max_error << "\n";
}

#ifdef BITRL_IRRLICHT
// The arm of section 6 in an Irrlicht window. The frames are drawn from our
// forward kinematics, the bodies are where Chrono puts them
void render(const SpatialArm &arm)
{
    using namespace bitrl::utils::render;

    chrono::ChSystemNSC system;
    const ChronoArm model = build_chrono_arm(system, arm);
    model.base->GetVisualShape(0)->SetColor(chrono::ChColor(0.30f, 0.30f, 0.30f));
    model.turret->GetVisualShape(0)->SetColor(chrono::ChColor(0.30f, 0.30f, 0.30f));
    model.upper_arm->GetVisualShape(0)->SetColor(chrono::ChColor(0.55f, 0.70f, 0.90f));
    model.forearm->GetVisualShape(0)->SetColor(chrono::ChColor(0.90f, 0.60f, 0.30f));

    chrono::irrlicht::ChVisualSystemIrrlicht vis;
    vis.AttachSystem(&system);
    vis.SetCameraVertical(chrono::CameraVerticalDir::Z);
    vis.SetWindowSize(1280, 720);
    vis.SetWindowTitle("Example 27: Forward kinematics");
    vis.Initialize();
    vis.AddCamera({1.3, -1.1, 0.9}, {0.0, 0.15, 0.25});
    vis.GetActiveCamera()->setFOV(0.75f);
    vis.AddTypicalLights();
    vis.BindAll();

    auto *readout = vis.GetGUIEnvironment()->addStaticText(
        L"", irr::core::rect<irr::s32>(1020, 10, 1270, 130), true, true);
    readout->setBackgroundColor(irr::video::SColor(220, 255, 255, 255));
    readout->setDrawBackground(true);

    // The path of the tip over the last few seconds
    std::vector<ChVector3d> trail;
    const std::size_t trail_length = 400;

    const real_t step_size = 0.001;
    const int steps_per_frame = 10;
    chrono::ChRealtimeStepTimer real_time;

    while (vis.Run())
    {
        const Joints q = model.joints();
        const auto frames = spatial_fk_all(arm, q);
        const ChVector3d simulated = model.tip();

        trail.push_back(simulated);
        if (trail.size() > trail_length)
        {
            trail.erase(trail.begin());
        }

        std::ostringstream text;
        text << "t = " << std::fixed << std::setprecision(1) << system.GetChTime() << " s\n\n"
             << "q1 = " << clean(q[0]) / DEG << "  q2 = " << clean(q[1]) / DEG
             << "  q3 = " << clean(q[2]) / DEG << " deg\n\n"
             << "tip from FK:  " << to_string(frames[3].GetPos()) << "\n"
             << "tip in Chrono:  " << to_string(simulated) << "\n"
             << "error = " << std::scientific << std::setprecision(1)
             << (simulated - frames[3].GetPos()).Length() << " m\n";
        readout->setText(irr::core::stringw(text.str().c_str()).c_str());

        vis.BeginScene();
        vis.Render();

        chrono::irrlicht::tools::drawGrid(&vis, 0.25, 0.25, 8, 8, chrono::ChCoordsysd(),
                                          chrono::ChColor(0.35f, 0.45f, 0.55f));
        irr_draw_frame(vis, ChFrame<>(), 0.3, "W");

        // Every joint turns around the blue z axis of its own frame
        // Joints 1 and 2 have the same origin, so the second label goes above it
        irr_draw_frame(vis, frames[0], 0.15, "1");
        irr_draw_frame(vis, frames[1], 0.15);
        irr_draw_label(vis, "2", frames[1].GetPos() + ChVector3d(0.0, 0.0, 0.05));
        irr_draw_frame(vis, frames[2], 0.15, "3");
        irr_draw_frame(vis, frames[3], 0.15, "E");

        for (std::size_t i = 1; i < trail.size(); ++i)
        {
            irr_draw_line(vis, trail[i - 1], trail[i], chrono::ChColor(1.0f, 0.8f, 0.2f), 2.0);
        }

        for (int i = 0; i < steps_per_frame; ++i)
        {
            system.DoStepDynamics(step_size);
        }
        vis.EndScene();
        real_time.Spin(steps_per_frame * step_size);
    }
}
#endif

} // namespace example_27

int main(int argc, char **argv)
{
    using namespace example_27;

    const PlanarArm planar;
    const SpatialArm spatial;

    one_joint();
    planar_arm(planar);
    workspace(planar);
    spatial_arm(spatial);
    denavit_hartenberg(spatial);
    check_in_simulation(spatial);

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

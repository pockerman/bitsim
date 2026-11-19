// A first look at CHRONO_DiffDriveRobot, the robot that the later examples drive. It is loaded from
// its robot file, put on the ground, and driven through a short script: straight ahead, a turn on
// the spot, an arc and a stop. Each command is a forward speed and a turning rate. The example
// compares where the simulated robot goes with where DiffDriveDynamics, the kinematic model of
// diff_drive_kinematics, says the commands should take it.
//
// Usage: chrono_diff_drive [--render] [--record DIR]
//
// --render shows the robot in a window, in real time. --record DIR writes the window to DIR, ten
// pictures a second.

#include "bitrl/bitrl_config.h"

#ifdef BITRL_CHRONO

#include "bitrl/bitrl_consts.h"
#include "bitrl/bitrl_types.h"
#include "bitrl/dynamics/diff_drive_dynamics.h"
#include "bitrl/rigid_bodies/chrono_robots/diff_drive_robot.h"
#include "bitrl/utils/io/print_utils.h"

#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChSystemNSC.h>

#ifdef BITRL_IRRLICHT
#include "bitrl/utils/render/irrlicht_scene.h"
#include "bitrl/utils/render/irrlicht_utils.h"
#include <chrono_irrlicht/ChIrrTools.h>
#include <chrono_irrlicht/ChVisualSystemIrrlicht.h>
#endif

#include <any>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace chrono_diff_drive
{

using bitrl::real_t;
using bitrl::uint_t;
using bitrl::dynamics::DiffDriveDynamics;
using bitrl::rb::bitrl_chrono::CHRONO_DiffDriveRobot;

const real_t PI = bitrl::consts::maths::PI;
const std::string ROBOT_FILE = bitrl::consts::ROBOTS_DIR + "/bitrl_diff_drive_robot.json";
const real_t STEP_SIZE = 0.002;  // [s]
const real_t PRINT_PERIOD = 0.5; // [s]
const real_t RECORD_RATE = 10.0; // [pictures/s]
const real_t RENDER_RATE = 30.0; // [frames/s]

// A part of the script: drive at v [m/s], turning at omega [rad/s], for duration [s]
struct Segment
{
    std::string name;
    real_t v;
    real_t omega;
    real_t duration;
};

const std::vector<Segment> SCRIPT{
    {"straight", 0.3, 0.0, 4.0},
    {"turn on the spot", 0.0, PI / 4.0, 2.0}, // a quarter of a turn
    {"arc", 0.3, 0.3, 2.0 * PI / 3.0 / 0.3},  // a third of a circle of radius 1 m
    {"stop", 0.0, 0.0, 1.5},
};

// Where the robot is: the centre of its axle and its heading
struct Pose
{
    real_t x;
    real_t y;
    real_t yaw;
};

class World
{
  public:
    World()
    {
        system_.SetGravitationalAcceleration(
            chrono::ChVector3d(0.0, 0.0, -bitrl::consts::maths::G));
        system_.SetCollisionSystemType(chrono::ChCollisionSystem::Type::BULLET);
        system_.SetSolverType(chrono::ChSolver::Type::BARZILAIBORWEIN);
        system_.GetSolver()->AsIterative()->SetMaxIterations(40);

        auto material = chrono_types::make_shared<chrono::ChContactMaterialNSC>();
        material->SetFriction(0.8f);
        material->SetRestitution(0.1f);
        ground_ = chrono_types::make_shared<chrono::ChBodyEasyBox>(10.0, 10.0, 0.1, 1000.0, true,
                                                                   true, material);
        ground_->SetPos(chrono::ChVector3d(0.0, 0.0, -0.05));
        ground_->SetFixed(true);
        ground_->GetVisualShape(0)->SetColor(chrono::ChColor(0.45f, 0.45f, 0.42f));
        system_.Add(ground_);

        robot_.load(std::filesystem::path(ROBOT_FILE));
        robot_.set_world_pose(0.0, 0.0, 0.0);
        robot_.add_to_sys(system_);
    }

    void step(real_t v, real_t omega)
    {
        // set_velocity turns (v, omega) into a target speed for each wheel's motor, and update
        // runs the motors' speed controllers and puts their torque on the wheels
        robot_.set_velocity(v, omega);
        robot_.update(STEP_SIZE);
        system_.DoStepDynamics(STEP_SIZE);
    }

    Pose pose() const
    {
        const auto c = robot_.axle_center();
        return {c.x(), c.y(), robot_.heading()};
    }

    // The speed of the chassis along its heading [m/s], and its turning rate [rad/s]
    std::array<real_t, 2> velocity() const
    {
        const auto chassis = robot_.get_chassis();
        const auto vel = chassis->GetPosDt();
        const real_t yaw = robot_.heading();
        return {vel.x() * std::cos(yaw) + vel.y() * std::sin(yaw), chassis->GetAngVelParent().z()};
    }

    real_t time() const { return system_.GetChTime(); }
    chrono::ChSystemNSC &system() { return system_; }
    const CHRONO_DiffDriveRobot &robot() const { return robot_; }

  private:
    chrono::ChSystemNSC system_;
    std::shared_ptr<chrono::ChBodyEasyBox> ground_;
    CHRONO_DiffDriveRobot robot_;
};

// The kinematic model: the pose that the commands give, if the wheels turned at once at the speed
// they are told and never slipped
class Kinematics
{
  public:
    Kinematics() : model_(DiffDriveDynamics::DynamicVersion::V1, false)
    {
        model_.set_time_step(STEP_SIZE);
    }

    void step(real_t v, real_t omega)
    {
        const std::map<std::string, std::any> input{{"v", std::any(v)},
                                                    {"w", std::any(omega)},
                                                    {"errors", std::any(std::array<real_t, 2>{})}};
        model_.integrate(input);
    }

    Pose pose() const
    {
        return {model_.get_x_position(), model_.get_y_position(), model_.get_orientation()};
    }

  private:
    DiffDriveDynamics model_;
};

real_t angle_diff(real_t a, real_t b) { return std::remainder(a - b, 2.0 * PI); }

#ifdef BITRL_IRRLICHT
class Display
{
  public:
    Display(World &world, const std::string &record) : world_(world), recorder_(record, RECORD_RATE)
    {
        vis_.AttachSystem(&world_.system());
        vis_.SetWindowSize(1280, 720);
        vis_.SetWindowTitle("A differential drive robot in Chrono");
        vis_.SetCameraVertical(chrono::CameraVerticalDir::Z);
        vis_.Initialize();
        vis_.AddCamera({0.45, -1.3, 1.25}, {0.45, 0.45, 0.0});
        // A soft light: the typical lights wash the colours out
        vis_.AddLightDirectional(60.0, 60.0, chrono::ChColor(0.45f, 0.45f, 0.45f),
                                 chrono::ChColor(0.05f, 0.05f, 0.05f),
                                 chrono::ChColor(0.55f, 0.55f, 0.55f));
    }

    // Returns false once the window is closed
    bool show(const Pose &truth, const Pose &model, const Segment &segment)
    {
        if (!vis_.Run())
        {
            return false;
        }
        truth_.push_back({truth.x, truth.y, 0.01});
        model_.push_back({model.x, model.y, 0.012});

        vis_.BeginScene();
        vis_.Render();
        // A grid of 25 cm squares, for scale
        chrono::irrlicht::tools::drawGrid(&vis_, 0.25, 0.25, 16, 16,
                                          chrono::ChCoordsysd(chrono::ChVector3d(0.0, 0.0, 0.002)),
                                          chrono::ChColor(0.6f, 0.6f, 0.6f));
        bitrl::utils::render::irr_draw_world_axes(vis_, 0.5);
        draw_trail_(model_, chrono::ChColor(0.2f, 0.8f, 0.9f));
        draw_trail_(truth_, chrono::ChColor(1.0f, 0.85f, 0.2f));

        const auto v = world_.velocity();
        std::ostringstream text;
        text << std::fixed << std::setprecision(2) << "t = " << world_.time() << " s   "
             << segment.name << ": v = " << segment.v << " m/s, omega = " << segment.omega
             << " rad/s   robot: v = " << v[0] << " m/s, omega = " << v[1]
             << " rad/s   off the model: " << std::setprecision(3)
             << std::hypot(truth.x - model.x, truth.y - model.y)
             << " m   yellow: the robot, cyan: the model";
        bitrl::utils::render::draw_status_bar(vis_, text.str());
        vis_.EndScene();

        recorder_.record(vis_, world_.time());
        return true;
    }

  private:
    void draw_trail_(const std::vector<chrono::ChVector3d> &trail, const chrono::ChColor &colour)
    {
        for (std::size_t i = 1; i < trail.size(); ++i)
        {
            bitrl::utils::render::irr_draw_line(vis_, trail[i - 1], trail[i], colour, 3.0);
        }
    }

    World &world_;
    chrono::irrlicht::ChVisualSystemIrrlicht vis_;
    bitrl::utils::render::FrameRecorder recorder_;
    std::vector<chrono::ChVector3d> truth_;
    std::vector<chrono::ChVector3d> model_;
};
#endif

struct Options
{
    bool render{false};
    std::string record;
};

int run(const Options &options)
{
    World world;
    Kinematics kinematics;

    const auto &data = world.robot().get_data();
    std::cout << std::fixed << std::setprecision(3) << "Robot: " << ROBOT_FILE << "\n"
              << "  wheels of radius " << data.wheel_radius << " m, " << data.axle_length
              << " m apart\n\n";

#ifdef BITRL_IRRLICHT
    std::optional<Display> display;
    if (options.render)
    {
        display.emplace(world, options.record);
    }
#else
    if (options.render)
    {
        std::cerr << "--render and --record need bitrl configured with BITRL_IRRLICHT=ON\n";
        return 1;
    }
#endif

    std::cout << "    t [s]  command            v [m/s]  w [rad/s]   x [m]   y [m]  yaw [deg]"
                 "   model x   model y  model yaw   off [m]\n";

    real_t max_off = 0.0;
    real_t next_print = 0.0;
    real_t next_frame = 0.0;
    real_t segment_end = 0.0;
    const auto wall_start = std::chrono::steady_clock::now();
    for (const auto &segment : SCRIPT)
    {
        segment_end += segment.duration;
        while (world.time() < segment_end - 0.5 * STEP_SIZE)
        {
            world.step(segment.v, segment.omega);
            kinematics.step(segment.v, segment.omega);

            const Pose truth = world.pose();
            const Pose model = kinematics.pose();
            const real_t off = std::hypot(truth.x - model.x, truth.y - model.y);
            max_off = std::max(max_off, off);

            if (world.time() + 1e-9 >= next_print)
            {
                next_print += PRINT_PERIOD;
                const auto v = world.velocity();
                std::cout << std::setw(9) << std::setprecision(2) << world.time() << "  "
                          << std::left << std::setw(17) << segment.name << std::right
                          << std::setw(9) << v[0] << std::setw(11) << v[1] << std::setprecision(3)
                          << std::setw(8) << truth.x << std::setw(8) << truth.y
                          << std::setprecision(1) << std::setw(11) << truth.yaw * 180.0 / PI
                          << std::setprecision(3) << std::setw(10) << model.x << std::setw(10)
                          << model.y << std::setprecision(1) << std::setw(11)
                          << model.yaw * 180.0 / PI << std::setprecision(3) << std::setw(10) << off
                          << "\n";
            }

#ifdef BITRL_IRRLICHT
            if (display && world.time() + 1e-9 >= next_frame)
            {
                next_frame += 1.0 / RENDER_RATE;
                if (!display->show(truth, model, segment))
                {
                    return 0;
                }
                const real_t ahead =
                    world.time() -
                    std::chrono::duration<real_t>(std::chrono::steady_clock::now() - wall_start)
                        .count();
                if (ahead > 0.0)
                {
                    std::this_thread::sleep_for(std::chrono::duration<real_t>(ahead));
                }
            }
#else
            (void)next_frame;
#endif
        }
    }

    const Pose truth = world.pose();
    const Pose model = kinematics.pose();
    std::cout << std::setprecision(3) << "\nAt the end the robot is "
              << std::hypot(truth.x - model.x, truth.y - model.y) << " m and "
              << std::setprecision(1) << std::abs(angle_diff(truth.yaw, model.yaw)) * 180.0 / PI
              << " deg off the kinematic model, " << std::setprecision(3) << max_off
              << " m at most\n";

    // The motors need a moment to reach a new speed, and overshoot it, so on the way the robot
    // is some centimetres off the model. By the end the errors mostly cancel: more than 10 cm
    // would mean that something is wrong with the robot or its file
    return std::hypot(truth.x - model.x, truth.y - model.y) < 0.1 ? 0 : 1;
}

} // namespace chrono_diff_drive

int main(int argc, char **argv)
{
    using namespace chrono_diff_drive;

    Options options;
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "--render")
        {
            options.render = true;
        }
        else if (arg == "--record" && i + 1 < argc)
        {
            options.render = true;
            options.record = argv[++i];
        }
        else
        {
            std::cerr << "Usage: chrono_diff_drive [--render] [--record DIR]\n";
            return 1;
        }
    }
    return run(options);
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

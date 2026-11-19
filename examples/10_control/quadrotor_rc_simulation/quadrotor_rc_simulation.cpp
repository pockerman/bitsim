// Flying a simulated quadrotor from the laptop, as with a toy drone's transmitter. The laptop
// sends RC commands, throttle, yaw, pitch and roll, over a simulated radio link; the drone's
// flight controller, in angle mode, turns them into rotor speeds; and QuadrotorDynamics flies it.
//
// Usage: quadrotor_rc_simulation [--render] [--script] [--failsafe] [--record DIR]
//
// Without --render it flies a scripted flight, headless: take off, fly forward, turn round, fly
// back and land, and it returns 0 if the drone took off, came back and landed gently. --render
// opens a window and you fly it from the keyboard; with --script as well the window shows the
// scripted flight. --failsafe stops the transmitter in the middle of the scripted flight, to show
// what the receiver does then. --record DIR writes the window to DIR, ten pictures a second.

#include "bitrl/bitrl_config.h"

#if defined(BITRL_CHRONO) && defined(BITRL_IRRLICHT)

#include "bitrl/bitrl_consts.h"
#include "bitrl/bitrl_types.h"
#include "bitrl/boards/sim_rc_link.h"
#include "bitrl/control/quadrotor_flight_controller.h"
#include "bitrl/control/rc_command.h"
#include "bitrl/dynamics/quadrotor_dynamics.h"
#include "bitrl/dynamics/system_state.h"
#include "bitrl/geometry/rotations.h"
#include "bitrl/utils/io/print_utils.h"
#include "bitrl/utils/render/irrlicht_scene.h"
#include "bitrl/utils/render/irrlicht_utils.h"
#include "bitrl/utils/render/keyboard_rc.h"

#include <chrono/assets/ChVisualShapeBox.h>
#include <chrono/assets/ChVisualShapeCylinder.h>
#include <chrono/physics/ChBody.h>
#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChSystemNSC.h>
#include <chrono_irrlicht/ChVisualSystemIrrlicht.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace quadrotor_rc_simulation
{

using bitrl::real_t;
using bitrl::RealColVec3d;
using bitrl::RealVec;
using bitrl::boards::SIM_RCLink;
using bitrl::control::QuadrotorFlightController;
using bitrl::control::RCCommand;
using bitrl::dynamics::QuadrotorDynamics;
using bitrl::dynamics::QuadrotorDynamicsConfig;
using bitrl::dynamics::SysState;
using bitrl::utils::io::clean;
using chrono::ChVector3d;

const real_t PI = bitrl::consts::maths::PI;
const real_t DEG = PI / 180.0;
const real_t STEP_SIZE = 0.001;  // [s] of the dynamics and the flight controller
const real_t SEND_PERIOD = 0.02; // [s] the transmitter sends 50 commands a second
const real_t PRINT_PERIOD = 0.5; // [s]
const real_t RENDER_RATE = 30.0; // [frames/s]
const real_t RECORD_RATE = 10.0; // [frames/s] written by --record
const real_t HARD_LANDING = 1.0; // [m/s] down, faster than this hurts the drone
const real_t FAILSAFE_AT = 7.0;  // [s] when --failsafe stops the transmitter

// The quadrotor of Luukkonen, Modelling and control of quadcopter, as in the
// quadrotor_dynamics example, with some air drag
QuadrotorDynamicsConfig quadrotor_config()
{
    QuadrotorDynamicsConfig config;
    config.dt = STEP_SIZE;
    config.mass = 0.468;
    config.l = 0.225;
    config.k_1 = 2.980e-6;
    config.k_2 = 1.140e-7;
    config.Jx = 4.856e-3;
    config.Jy = 4.856e-3;
    config.Jz = 8.801e-3;
    config.drag = 0.25;
    return config;
}

SysState<12> on_the_ground()
{
    std::array<std::pair<std::string, real_t>, 12> values{{{"x", 0.0},
                                                           {"y", 0.0},
                                                           {"z", 0.0},
                                                           {"u", 0.0},
                                                           {"v", 0.0},
                                                           {"w", 0.0},
                                                           {"p", 0.0},
                                                           {"q", 0.0},
                                                           {"r", 0.0},
                                                           {"phi", 0.0},
                                                           {"theta", 0.0},
                                                           {"psi", 0.0}}};
    return SysState<12>(std::move(values));
}

// ---------------------------------------------------------------------------
// The drone: its receiver, its flight controller and its dynamics
// ---------------------------------------------------------------------------

class SimulatedQuadrotor
{
  public:
    SimulatedQuadrotor()
        : config_(quadrotor_config()), state_(on_the_ground()), dynamics_(config_, state_),
          controller_(config_)
    {
    }

    // The transmitter end of the radio
    SIM_RCLink &radio() { return radio_; }
    const SIM_RCLink &radio() const { return radio_; }

    // One step: the receiver gives the latest command, the flight controller sets the rotors,
    // and the dynamics move the drone. The floor stops it
    void step(real_t time)
    {
        command_ = radio_.receive(time);
        rotors_ = controller_.rotor_speeds(command_, dynamics_);
        dynamics_.integrate(rotors_);
        stand_on_the_floor_();
    }

    const QuadrotorDynamics &dynamics() const { return dynamics_; }
    const QuadrotorFlightController &controller() const { return controller_; }
    const RCCommand &command() const { return command_; }
    const RealVec &rotors() const { return rotors_; }

    real_t height() const { return -dynamics_.get_position()[2]; }
    bool on_the_floor() const { return on_floor_; }
    int n_hard_landings() const { return n_hard_landings_; }
    real_t last_touchdown_speed() const { return touchdown_speed_; }

  private:
    // NED: z is down, so the floor is at z = 0 and the drone is below it when z > 0. On the floor
    // it stands still and level, facing where it faced
    void stand_on_the_floor_()
    {
        auto &s = dynamics_.get_state();
        if (s.get("z") < 0.0)
        {
            on_floor_ = false;
            return;
        }
        if (!on_floor_)
        {
            // Touching down: how fast was it coming down, in the world?
            touchdown_speed_ = std::abs(dynamics_.get_velocity()[2]);
            if (touchdown_speed_ > HARD_LANDING)
            {
                ++n_hard_landings_;
            }
        }
        on_floor_ = true;
        for (const char *name : {"z", "u", "v", "w", "p", "q", "r", "phi", "theta"})
        {
            s.set(name, 0.0);
        }
    }

    QuadrotorDynamicsConfig config_;
    SysState<12> state_;
    QuadrotorDynamics dynamics_;
    QuadrotorFlightController controller_;
    SIM_RCLink radio_{0.5};
    RCCommand command_{};
    RealVec rotors_{RealVec::Zero(4)};
    bool on_floor_{true};
    int n_hard_landings_{0};
    real_t touchdown_speed_{0.0};
};

// ---------------------------------------------------------------------------
// The scripted flight
// ---------------------------------------------------------------------------

struct Phase
{
    real_t until; // [s]
    std::string name;
    RCCommand sticks; // the throttle 0 means: the hover throttle plus `throttle_offset`
    real_t throttle_offset;
};

// Take off, fly forward, turn round, fly back and land. The throttle is given relative to the
// hover throttle, as a pilot feels it
std::vector<Phase> script()
{
    const auto at = [](real_t until, const std::string &name, real_t throttle_offset, real_t yaw,
                       real_t pitch, real_t roll, bool armed = true) {
        return Phase{until, name, RCCommand{0.0, yaw, pitch, roll, armed}, throttle_offset};
    };
    return {
        at(0.5, "on the floor", -0.5, 0.0, 0.0, 0.0, false),
        at(1.0, "arm", -0.5, 0.0, 0.0, 0.0),
        at(2.5, "take off", 0.06, 0.0, 0.0, 0.0),
        at(3.5, "level off", -0.05, 0.0, 0.0, 0.0),
        at(4.5, "hover", 0.0, 0.0, 0.0, 0.0),
        at(6.5, "forward", 0.02, 0.0, 0.6, 0.0),
        at(8.5, "brake", 0.02, 0.0, -0.6, 0.0),
        at(10.0, "turn round", 0.0, 1.0, 0.0, 0.0),
        at(12.0, "forward", 0.02, 0.0, 0.6, 0.0),
        at(14.0, "brake", 0.02, 0.0, -0.6, 0.0),
        at(15.5, "hover", 0.0, 0.0, 0.0, 0.0),
        at(22.0, "land", -0.02, 0.0, 0.0, 0.0),
        at(23.0, "disarm", -0.5, 0.0, 0.0, 0.0, false),
    };
}

class ScriptedPilot
{
  public:
    explicit ScriptedPilot(real_t hover_throttle) : hover_(hover_throttle), phases_(script()) {}

    RCCommand command(real_t time) const
    {
        const Phase &p = phase(time);
        RCCommand command = p.sticks;
        command.throttle = std::clamp(hover_ + p.throttle_offset, 0.0, 1.0);
        return command;
    }

    const Phase &phase(real_t time) const
    {
        for (const auto &p : phases_)
        {
            if (time < p.until)
            {
                return p;
            }
        }
        return phases_.back();
    }

    real_t duration() const { return phases_.back().until; }

  private:
    real_t hover_;
    std::vector<Phase> phases_;
};

// ---------------------------------------------------------------------------
// The window
// ---------------------------------------------------------------------------

// NED to Chrono's z up: (x, y, z) -> (x, -y, -z), a half turn about x. The same turn takes the
// body's NED axes to the drawing's
const chrono::ChMatrix33d NED_TO_Z_UP(ChVector3d(1, 0, 0), ChVector3d(0, -1, 0),
                                      ChVector3d(0, 0, -1));

ChVector3d to_z_up(const RealColVec3d &p) { return {p[0], -p[1], -p[2]}; }

class Display
{
  public:
    Display(const std::string &title, const std::string &record) : recorder_(record, RECORD_RATE)
    {
        system_.SetGravitationalAcceleration(ChVector3d(0, 0, 0));
        // A floor of 2 m tiles, so that the window shows how fast the drone moves
        const int n_tiles = 20;
        const real_t tile = 2.0;
        for (int i = 0; i < n_tiles; ++i)
        {
            for (int j = 0; j < n_tiles; ++j)
            {
                auto t = chrono_types::make_shared<chrono::ChBodyEasyBox>(tile, tile, 0.1, 1000.0,
                                                                          true, false);
                t->SetPos(ChVector3d((i - n_tiles / 2 + 0.5) * tile, (j - n_tiles / 2 + 0.5) * tile,
                                     -0.05));
                t->SetFixed(true);
                t->GetVisualShape(0)->SetColor((i + j) % 2 == 0
                                                   ? chrono::ChColor(0.45f, 0.58f, 0.40f)
                                                   : chrono::ChColor(0.37f, 0.50f, 0.33f));
                system_.Add(t);
            }
        }

        // Posts to see the drone move against, and the start pad
        for (const auto &p :
             std::vector<ChVector3d>{{4, 4, 0}, {4, -4, 0}, {-4, 4, 0}, {-4, -4, 0}, {10, 0, 0}})
        {
            auto post =
                chrono_types::make_shared<chrono::ChBodyEasyBox>(0.2, 0.2, 2.0, 100, true, false);
            post->SetPos(p + ChVector3d(0, 0, 1.0));
            post->SetFixed(true);
            post->GetVisualShape(0)->SetColor(chrono::ChColor(0.85f, 0.55f, 0.15f));
            system_.Add(post);
        }
        auto pad =
            chrono_types::make_shared<chrono::ChBodyEasyBox>(0.8, 0.8, 0.01, 100, true, false);
        pad->SetPos(ChVector3d(0, 0, 0.005));
        pad->SetFixed(true);
        pad->GetVisualShape(0)->SetColor(chrono::ChColor(0.9f, 0.9f, 0.9f));
        system_.Add(pad);

        drone_ = make_drone_();
        system_.Add(drone_);

        vis_.AttachSystem(&system_);
        vis_.SetWindowSize(1280, 720);
        vis_.SetWindowTitle(title);
        vis_.SetCameraVertical(chrono::CameraVerticalDir::Z);
        vis_.Initialize();
        vis_.AddCamera({-2.5, 0.0, 1.2}, {0.0, 0.0, 0.3});
        // A soft light: the typical lights wash the scene out
        vis_.AddLightDirectional(60.0, 60.0, chrono::ChColor(0.45f, 0.45f, 0.45f),
                                 chrono::ChColor(0.05f, 0.05f, 0.05f),
                                 chrono::ChColor(0.55f, 0.55f, 0.55f));
    }

    chrono::irrlicht::ChVisualSystemIrrlicht &vis() { return vis_; }

    // Returns false once the window is closed
    bool show(const SimulatedQuadrotor &drone, real_t time, const std::string &status)
    {
        if (!vis_.Run())
        {
            return false;
        }
        const auto &d = drone.dynamics();
        const auto R = bitrl::geometry::from_yaw_pitch_roll({d.psi(), d.theta(), d.phi()});
        const ChVector3d position = to_z_up(d.get_position());
        drone_->SetPos(position);
        drone_->SetRot(chrono::ChMatrix33d(NED_TO_Z_UP * R * NED_TO_Z_UP));
        trail_.add(position.x(), position.y());

        // The camera follows behind the drone, along its heading
        const ChVector3d behind(-std::cos(-d.psi()) * 2.5, -std::sin(-d.psi()) * 2.5, 1.0);
        vis_.UpdateCamera(position + behind, position);

        vis_.BeginScene();
        vis_.Render();
        trail_.draw(vis_, chrono::ChColor(1.0f, 0.9f, 0.2f), 1.5);
        bitrl::utils::render::irr_draw_line(vis_, position,
                                            ChVector3d(position.x(), position.y(), 0),
                                            chrono::ChColor(0.2f, 0.2f, 0.2f), 1.0);
        bitrl::utils::render::draw_status_bar(vis_, status);
        vis_.EndScene();
        recorder_.record(vis_, time);
        return true;
    }

  private:
    // Two arms in a cross, a body, and the four rotors; the front one is red
    std::shared_ptr<chrono::ChBody> make_drone_()
    {
        auto body = chrono_types::make_shared<chrono::ChBody>();
        body->SetFixed(true);
        const real_t arm = 0.225;
        const auto dark = chrono::ChColor(0.15f, 0.15f, 0.18f);
        auto add = [&](std::shared_ptr<chrono::ChVisualShape> shape, const ChVector3d &at,
                       const chrono::ChColor &colour)
        {
            shape->SetColor(colour);
            body->AddVisualShape(shape, chrono::ChFrame<>(at));
        };
        add(chrono_types::make_shared<chrono::ChVisualShapeBox>(2 * arm, 0.03, 0.02), {0, 0, 0},
            dark);
        add(chrono_types::make_shared<chrono::ChVisualShapeBox>(0.03, 2 * arm, 0.02), {0, 0, 0},
            dark);
        add(chrono_types::make_shared<chrono::ChVisualShapeBox>(0.1, 0.1, 0.05), {0, 0, 0},
            chrono::ChColor(0.25f, 0.45f, 0.8f));
        // Rotors 1 front, 2 right, 3 back, 4 left; in the drawing's axes right is -y
        const std::array<ChVector3d, 4> rotors{ChVector3d(arm, 0, 0.02), ChVector3d(0, -arm, 0.02),
                                               ChVector3d(-arm, 0, 0.02), ChVector3d(0, arm, 0.02)};
        for (std::size_t i = 0; i < rotors.size(); ++i)
        {
            auto disc = chrono_types::make_shared<chrono::ChVisualShapeCylinder>(0.09, 0.005);
            add(disc, rotors[i],
                i == 0 ? chrono::ChColor(0.9f, 0.15f, 0.1f) : chrono::ChColor(0.75f, 0.75f, 0.75f));
        }
        return body;
    }

    chrono::ChSystemNSC system_;
    std::shared_ptr<chrono::ChBody> drone_;
    chrono::irrlicht::ChVisualSystemIrrlicht vis_;
    bitrl::utils::render::FrameRecorder recorder_;
    bitrl::utils::render::Trail trail_{0.05, 0.0};
};

// ---------------------------------------------------------------------------
// Running it
// ---------------------------------------------------------------------------

struct Options
{
    bool render{false};
    bool script{false};
    bool failsafe{false};
    std::string record;
};

Options parse(int argc, char **argv)
{
    Options options;
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "--render")
        {
            options.render = true;
        }
        else if (arg == "--script")
        {
            options.script = true;
        }
        else if (arg == "--failsafe")
        {
            options.failsafe = true;
        }
        else if (arg == "--record" && i + 1 < argc)
        {
            options.record = argv[++i];
        }
        else
        {
            std::cout << "Usage: quadrotor_rc_simulation [--render] [--script] [--failsafe] "
                         "[--record DIR]\n"
                      << "  --render     fly it from the keyboard in a window\n"
                      << "  --script     with --render, watch the scripted flight instead\n"
                      << "  --failsafe   stop the transmitter at " << FAILSAFE_AT
                      << " s of the scripted flight\n"
                      << "  --record DIR write the window to DIR\n"
                      << "Keys: Space arm/disarm, W/S throttle, A/D yaw, arrows pitch and roll, "
                         "R start again"
                      << std::endl;
            std::exit(arg == "--help" || arg == "-h" ? 0 : 1);
        }
    }
    if (!options.render)
    {
        options.script = true;
    }
    return options;
}

std::string status_line(const SimulatedQuadrotor &drone, real_t time, const std::string &phase)
{
    const auto &d = drone.dynamics();
    const auto &c = drone.command();
    const RealColVec3d p = d.get_position();
    std::ostringstream out;
    out << std::fixed << std::setprecision(1) << "t " << time << " s   "
        << (drone.radio().in_failsafe(time) ? "FAILSAFE" : (c.armed ? "armed" : "disarmed"))
        << "   throttle " << std::setprecision(2) << c.throttle << " (hover "
        << drone.controller().hover_throttle() << ")   yaw " << c.yaw << " pitch " << c.pitch
        << " roll " << c.roll << "   height " << drone.height() << " m   x " << p[0] << " y "
        << p[1] << " m   heading " << std::setprecision(0) << d.psi() / DEG << " deg   " << phase;
    return out.str();
}

int run(const Options &options)
{
    auto drone_ptr = std::make_unique<SimulatedQuadrotor>();
    const ScriptedPilot pilot(drone_ptr->controller().hover_throttle());
    std::optional<Display> display;
    bitrl::utils::render::KeyboardRC keyboard;
    if (options.render)
    {
        display.emplace("Flying a quadrotor from the laptop", options.record);
        if (!options.script)
        {
            display->vis().AddUserEventReceiver(&keyboard);
        }
    }

    std::cout << std::fixed << std::setprecision(2) << "Quadrotor: " << quadrotor_config().mass
              << " kg, arms " << quadrotor_config().l << " m, rotors up to " << std::setprecision(0)
              << drone_ptr->controller().max_rotor_speed() << " rad/s, hover throttle "
              << std::setprecision(2) << drone_ptr->controller().hover_throttle() << "\n"
              << "Radio: a command every " << SEND_PERIOD * 1000 << " ms, failsafe after "
              << drone_ptr->radio().timeout() << " s without one\n"
              << (options.script ? "Flying the script\n" : "Flying from the keyboard\n")
              << "\n    t [s]   x [m]   y [m]  height [m]  roll  pitch  heading [deg]  throttle"
                 "  phase\n";

    real_t time = 0.0;
    real_t next_send = 0.0;
    real_t next_print = 0.0;
    real_t next_frame = 0.0;
    real_t max_height = 0.0;
    real_t max_tilt = 0.0;
    real_t max_distance = 0.0;
    bool flew = false;
    const real_t limit = options.script ? pilot.duration() + 1.0 : 1.0e9;
    const auto wall_start = std::chrono::steady_clock::now();

    while (time < limit)
    {
        const std::string phase = options.script ? pilot.phase(time).name : "keyboard";

        // The transmitter: the script or the keyboard, 50 times a second, unless --failsafe has
        // stopped it
        if (time + 1e-9 >= next_send)
        {
            next_send += SEND_PERIOD;
            const bool silent = options.failsafe && time >= FAILSAFE_AT;
            if (!silent)
            {
                drone_ptr->radio().send(
                    options.script ? pilot.command(time) : keyboard.command(SEND_PERIOD), time);
            }
        }

        drone_ptr->step(time);
        time += STEP_SIZE;

        const auto &d = drone_ptr->dynamics();
        max_height = std::max(max_height, drone_ptr->height());
        max_tilt = std::max({max_tilt, std::abs(d.phi()), std::abs(d.theta())});
        max_distance = std::max(max_distance, std::hypot(d.get_position()[0], d.get_position()[1]));
        flew = flew || drone_ptr->height() > 0.2;

        if (time + 1e-9 >= next_print)
        {
            next_print += PRINT_PERIOD;
            const RealColVec3d p = d.get_position();
            std::cout << std::setw(9) << std::setprecision(1) << time << std::setprecision(2)
                      << std::setw(8) << clean(p[0]) << std::setw(8) << clean(p[1]) << std::setw(12)
                      << clean(drone_ptr->height()) << std::setprecision(1) << std::setw(6)
                      << clean(d.phi() / DEG) << std::setw(7) << clean(d.theta() / DEG)
                      << std::setw(15) << clean(d.psi() / DEG) << std::setprecision(2)
                      << std::setw(10) << drone_ptr->command().throttle << "  "
                      << (drone_ptr->radio().in_failsafe(time) ? "FAILSAFE" : phase) << std::endl;
        }

        if (display && time + 1e-9 >= next_frame)
        {
            next_frame += 1.0 / RENDER_RATE;
            if (keyboard.reset_requested())
            {
                drone_ptr = std::make_unique<SimulatedQuadrotor>();
                keyboard.reset();
            }
            if (!display->show(*drone_ptr, time, status_line(*drone_ptr, time, phase)))
            {
                break;
            }
            const real_t ahead =
                time - std::chrono::duration<real_t>(std::chrono::steady_clock::now() - wall_start)
                           .count();
            if (ahead > 0.0)
            {
                std::this_thread::sleep_for(std::chrono::duration<real_t>(ahead));
            }
        }
    }

    const RealColVec3d p = drone_ptr->dynamics().get_position();
    const real_t from_start = std::hypot(p[0], p[1]);
    std::cout << std::setprecision(2) << "\nHighest " << max_height << " m, furthest "
              << max_distance << " m from the start, most tilt " << std::setprecision(1)
              << max_tilt / DEG << " deg\n"
              << std::setprecision(2) << "Ended "
              << (drone_ptr->on_the_floor() ? "on the floor" : "in the air") << ", " << from_start
              << " m from the start; touched down at " << drone_ptr->last_touchdown_speed()
              << " m/s; hard landings: " << drone_ptr->n_hard_landings() << std::endl;

    if (!options.script)
    {
        return 0;
    }
    const bool ok = flew && drone_ptr->on_the_floor() && drone_ptr->n_hard_landings() == 0 &&
                    from_start < 1.0 &&
                    max_tilt < drone_ptr->controller().parameters().max_tilt + 0.05;
    return options.failsafe ? 0 : (ok ? 0 : 1);
}

} // namespace quadrotor_rc_simulation

int main(int argc, char **argv)
{
    try
    {
        return quadrotor_rc_simulation::run(quadrotor_rc_simulation::parse(argc, argv));
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}

#else

#include <iostream>

int main()
{
    std::cerr << "quadrotor_rc_simulation needs bitrl configured with Chrono and Irrlicht"
              << std::endl;
    return 1;
}

#endif

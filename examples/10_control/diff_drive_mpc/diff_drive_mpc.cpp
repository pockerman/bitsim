// A differential drive robot in Chrono follows a figure of eight with model predictive control,
// whose wheels cannot turn faster than 0.8 m/s.
//
// Usage: diff_drive_mpc [--render] [--record DIR] [--laps N]
//
// MPC plans the commands of the two wheels over the next second, with a model of the robot, its
// motors' lag included, linearised along where it is expected to go, and the limit of the wheels.
// The other robot runs the same controller without the limit, a linear quadratic regulator along
// the path, and its commands are clipped to the limit afterwards. The path is faster than the
// wheels allow in places. --render shows both in a window, in real time; --record DIR also writes
// the window to DIR, ten pictures a second. --laps N drives N laps, 2 by default.
#include "bitsim/bitsim_config.h"

#ifdef BITSIM_CHRONO

#include "bitsim/bitsim_consts.h"
#include "bitsim/bitsim_types.h"
#include "bitsim/control/linear_mpc.h"
#include "bitsim/rigid_bodies/chrono_robots/diff_drive_robot.h"

#ifdef BITSIM_IRRLICHT
#include "bitsim/utils/render/irrlicht_scene.h"
#include "bitsim/utils/render/irrlicht_utils.h"

#include <chrono_irrlicht/ChVisualSystemIrrlicht.h>
#endif

#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChContactMaterialNSC.h>
#include <chrono/physics/ChSystemNSC.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <limits>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace diff_drive_mpc
{

using bitsim::real_t;
using bitsim::uint_t;
using bitsim::control::LinearMPC;
using bitsim::control::Matrix;
using bitsim::control::Vector;
using bitsim::rb::bitsim_chrono::CHRONO_DiffDriveRobot;

const real_t PI = bitsim::consts::maths::PI;
const std::string ROBOT_FILE = bitsim::consts::ROBOTS_DIR + "/bitsim_diff_drive_robot.json";
const real_t STEP_SIZE = 0.005;     // [s] of Chrono
const real_t CONTROL_PERIOD = 0.05; // [s] both controllers plan 20 times a second
const uint_t HORIZON = 20;          // the steps they plan, 1 s
const real_t MAX_WHEEL_SPEED = 0.8; // how fast a wheel can roll the robot [m/s]
const real_t TAU = 0.4;             // how long the motors take to reach a new speed [s]
const real_t LAP_TIME = 11.0;       // of the figure of eight [s]
const real_t SETTLE = 4.0;          // the errors before this are those of the start [s]
const real_t SEPARATION = 3.6;      // the second robot's figure is 3.6 m further along x [m]
const real_t PRINT_PERIOD = 2.0;    // [s]
const real_t RENDER_RATE = 30.0;    // [frames/s]
const real_t RECORD_RATE = 10.0;    // [frames/s]

// The figure of eight: x = 1.2 sin(phi), y = 0.6 sin(2 phi), once round in LAP_TIME
struct Path
{
    real_t a{1.2};
    real_t b{0.6};

    real_t phi(real_t t) const { return 2.0 * PI * t / LAP_TIME; }
    real_t dphi() const { return 2.0 * PI / LAP_TIME; }

    std::array<real_t, 2> point(real_t t) const
    {
        return {a * std::sin(phi(t)), b * std::sin(2.0 * phi(t))};
    }

    std::array<real_t, 2> velocity(real_t t) const
    {
        return {a * std::cos(phi(t)) * dphi(), 2.0 * b * std::cos(2.0 * phi(t)) * dphi()};
    }

    std::array<real_t, 2> acceleration(real_t t) const
    {
        const real_t w = dphi();
        return {-a * std::sin(phi(t)) * w * w, -4.0 * b * std::sin(2.0 * phi(t)) * w * w};
    }

    // The pose along the path, with the heading taken from the velocity and unwrapped
    Vector state(real_t t) const
    {
        const auto p = point(t);
        const auto v = velocity(t);
        Vector x(3);
        x << p[0], p[1], std::atan2(v[1], v[0]);
        return x;
    }

    // The wheel speeds that drive along the path: the speed and the turning rate
    // omega = (vx ay - vy ax) / |v|^2
    Vector wheels(real_t t, real_t axle) const
    {
        const auto v = velocity(t);
        const auto acc = acceleration(t);
        const real_t speed = std::hypot(v[0], v[1]);
        const real_t omega = (v[0] * acc[1] - v[1] * acc[0]) / (speed * speed);
        Vector u(2);
        u << speed - 0.5 * axle * omega, speed + 0.5 * axle * omega;
        return u;
    }
};

// Bring an angle within pi of a reference
real_t near(real_t angle, real_t reference)
{
    return reference + std::remainder(angle - reference, 2.0 * PI);
}

// The model of the robot over one control period. The state is the pose and the speeds the two
// wheels roll at, [x, y, theta, left, right]; the input is the speeds the motors are told, the
// command. The motors reach their command with a lag, here the time constant TAU:
//   left' = left + dt (command_left - left) / TAU, and the same for the right,
// and the robot drives with what the wheels do, v = (left + right) / 2 and
// omega = (right - left) / axle:
//   x' = x + dt v cos(theta), y' = y + dt v sin(theta), theta' = theta + dt omega.
// Linearised about (x, u), with the affine term c = f(x, u) - A x - B u
struct LinearStep
{
    Matrix A;
    Matrix B;
    Vector c;
};

Vector model_step(const Vector &x, const Vector &u, real_t axle)
{
    const real_t dt = CONTROL_PERIOD;
    const real_t v = 0.5 * (x[3] + x[4]);
    const real_t omega = (x[4] - x[3]) / axle;
    Vector f(5);
    f << x[0] + dt * v * std::cos(x[2]), x[1] + dt * v * std::sin(x[2]), x[2] + dt * omega,
        x[3] + dt * (u[0] - x[3]) / TAU, x[4] + dt * (u[1] - x[4]) / TAU;
    return f;
}

LinearStep linearise(const Vector &x, const Vector &u, real_t axle)
{
    const real_t dt = CONTROL_PERIOD;
    const real_t th = x[2];
    const real_t v = 0.5 * (x[3] + x[4]);

    LinearStep step{Matrix::Identity(5, 5), Matrix::Zero(5, 2), Vector()};
    step.A(0, 2) = -dt * v * std::sin(th);
    step.A(1, 2) = dt * v * std::cos(th);
    step.A(0, 3) = step.A(0, 4) = 0.5 * dt * std::cos(th);
    step.A(1, 3) = step.A(1, 4) = 0.5 * dt * std::sin(th);
    step.A(2, 3) = -dt / axle;
    step.A(2, 4) = dt / axle;
    step.A(3, 3) = step.A(4, 4) = 1.0 - dt / TAU;
    step.B(3, 0) = step.B(4, 1) = dt / TAU;
    step.c = model_step(x, u, axle) - step.A * x - step.B * u;
    return step;
}

// A path follower: model predictive control along the path, with or without the limit of the
// wheels. Without it, it is a linear quadratic regulator along the path
class Follower
{
  public:
    Follower(bool limited, real_t axle) : limited_(limited), axle_(axle), mpc_(5, 2, HORIZON)
    {
        Vector q(5);
        q << 20.0, 20.0, 2.0, 0.1, 0.1;
        mpc_.set_weights(q.asDiagonal().toDenseMatrix(), 0.5 * Matrix::Identity(2, 2));
        if (limited_)
        {
            mpc_.set_input_bounds(Vector::Constant(2, -MAX_WHEEL_SPEED),
                                  Vector::Constant(2, MAX_WHEEL_SPEED));
        }
    }

    // The commands of the wheels [m/s] for the robot in state x, its pose and the speeds of its
    // wheels, at time t on the path
    Vector control(const Path &path, real_t t, const Vector &x)
    {
        // The model is linearised about where the robot is expected to go, not about the path:
        // far from the path, facing the wrong way say, the path's heading is a poor place to
        // linearise cos and sin about. The expected trajectory is the model run from the pose
        // now with the inputs of the last plan, a step on; the first time, those of the path
        std::vector<Vector> u_nominal;
        for (uint_t k = 0; k < HORIZON; ++k)
        {
            if (last_inputs_.size() == HORIZON)
            {
                u_nominal.push_back(last_inputs_[std::min<uint_t>(k + 1, HORIZON - 1)]);
            }
            else
            {
                u_nominal.push_back(path.wheels(t + k * CONTROL_PERIOD, axle_));
            }
        }

        std::vector<Matrix> As;
        std::vector<Matrix> Bs;
        std::vector<Vector> cs;
        std::vector<Vector> x_ref;
        std::vector<Vector> u_ref;
        Vector x_nominal = x;
        for (uint_t k = 0; k < HORIZON; ++k)
        {
            const real_t tk = t + k * CONTROL_PERIOD;
            const auto step = linearise(x_nominal, u_nominal[k], axle_);
            As.push_back(step.A);
            Bs.push_back(step.B);
            cs.push_back(step.c);
            x_nominal = model_step(x_nominal, u_nominal[k], axle_);

            // the path's pose at the end of the step, its heading within pi of the robot's, and
            // the speeds of the wheels that drive along it
            const Vector pose = path.state(tk + CONTROL_PERIOD);
            const Vector wheels = path.wheels(tk + CONTROL_PERIOD, axle_);
            Vector next(5);
            next << pose[0], pose[1], near(pose[2], x_nominal[2]), wheels[0], wheels[1];
            x_ref.push_back(next);
            u_ref.push_back(path.wheels(tk, axle_));
        }
        mpc_.set_model(As, Bs, cs);
        mpc_.set_reference(x_ref, u_ref);

        const Vector u = mpc_.solve(x);
        last_inputs_ = mpc_.predicted_inputs();
        plan_.clear();
        for (const auto &xp : mpc_.predicted_states())
        {
            plan_.push_back({xp[0], xp[1]});
        }
        // the motors cannot do more, whatever is asked
        return u.cwiseMax(-MAX_WHEEL_SPEED).cwiseMin(MAX_WHEEL_SPEED);
    }

    const std::vector<std::array<real_t, 2>> &plan() const { return plan_; }
    bool limited() const { return limited_; }
    const LinearMPC &mpc() const { return mpc_; }

  private:
    bool limited_;
    real_t axle_;
    LinearMPC mpc_;
    std::vector<Vector> last_inputs_;
    std::vector<std::array<real_t, 2>> plan_;
};

// The world: a floor and the two robots
class World
{
  public:
    World()
    {
        system_.SetGravitationalAcceleration(
            chrono::ChVector3d(0.0, 0.0, -bitsim::consts::maths::G));
        system_.SetCollisionSystemType(chrono::ChCollisionSystem::Type::BULLET);
        system_.SetSolverType(chrono::ChSolver::Type::BARZILAIBORWEIN);
        system_.GetSolver()->AsIterative()->SetMaxIterations(40);

        auto material = chrono_types::make_shared<chrono::ChContactMaterialNSC>();
        material->SetFriction(0.8f);
        material->SetRestitution(0.1f);
        auto ground = chrono_types::make_shared<chrono::ChBodyEasyBox>(12.0, 6.0, 0.1, 1000.0, true,
                                                                       true, material);
        ground->SetPos(chrono::ChVector3d(0.5 * SEPARATION, 0.0, -0.05));
        ground->SetFixed(true);
        ground->GetVisualShape(0)->SetColor(chrono::ChColor(0.45f, 0.45f, 0.42f));
        system_.Add(ground);

        for (uint_t i = 0; i < 2; ++i)
        {
            robots_[i].load(std::filesystem::path(ROBOT_FILE));
        }
    }

    // Put robot i at its offset plus the pose
    void place(uint_t i, real_t x, real_t y, real_t yaw)
    {
        robots_[i].set_world_pose(x + offset(i), y, yaw);
    }

    void add_robots()
    {
        for (auto &robot : robots_)
        {
            robot.add_to_sys(system_);
        }
    }

    static real_t offset(uint_t i) { return i * SEPARATION; }

    // The pose of robot i relative to its own figure: (x, y, heading)
    Vector pose(uint_t i) const
    {
        const auto c = robots_[i].axle_center();
        Vector x(3);
        x << c.x() - offset(i), c.y(), robots_[i].heading();
        return x;
    }

    // The state of robot i for the controllers: its pose and the speeds its wheels roll at
    Vector state(uint_t i) const
    {
        const real_t r = robots_[i].get_data().wheel_radius;
        Vector x(5);
        x.head(3) = pose(i);
        x[3] = robots_[i].get_motor(CHRONO_DiffDriveRobot::LEFT)->speed() * r;
        x[4] = robots_[i].get_motor(CHRONO_DiffDriveRobot::RIGHT)->speed() * r;
        return x;
    }

    void set_wheels(uint_t i, const Vector &u)
    {
        const real_t r = robots_[i].get_data().wheel_radius;
        robots_[i].set_motor_speed(CHRONO_DiffDriveRobot::LEFT, u[0] / r);
        robots_[i].set_motor_speed(CHRONO_DiffDriveRobot::RIGHT, u[1] / r);
    }

    void step()
    {
        for (auto &robot : robots_)
        {
            robot.update(STEP_SIZE);
        }
        system_.DoStepDynamics(STEP_SIZE);
    }

    real_t time() const { return system_.GetChTime(); }
    chrono::ChSystemNSC &system() { return system_; }
    const CHRONO_DiffDriveRobot &robot(uint_t i) const { return robots_[i]; }

  private:
    chrono::ChSystemNSC system_;
    std::array<CHRONO_DiffDriveRobot, 2> robots_;
};

// How far the robot is from the path: the distance to its nearest point
real_t distance_to_path(const Path &path, const Vector &x)
{
    real_t best = std::numeric_limits<real_t>::infinity();
    for (int k = 0; k < 800; ++k)
    {
        const auto p = path.point(LAP_TIME * k / 800.0);
        best = std::min(best, std::hypot(x[0] - p[0], x[1] - p[1]));
    }
    return best;
}

#ifdef BITSIM_IRRLICHT
class Display
{
  public:
    Display(World &world, const Path &path, const std::string &record)
        : world_(world), recorder_(record, RECORD_RATE)
    {
        for (int k = 0; k <= 400; ++k)
        {
            const auto p = path.point(LAP_TIME * k / 400.0);
            figure_.push_back({p[0], p[1]});
        }
        vis_.AttachSystem(&world_.system());
        vis_.SetWindowSize(1280, 720);
        vis_.SetWindowTitle("Path following with model predictive control");
        vis_.SetCameraVertical(chrono::CameraVerticalDir::Z);
        vis_.Initialize();
        vis_.AddCamera({0.5 * SEPARATION, -2.4, 2.6}, {0.5 * SEPARATION, 0.15, 0.0});
        vis_.AddLightDirectional(60.0, 60.0, chrono::ChColor(0.45f, 0.45f, 0.45f),
                                 chrono::ChColor(0.05f, 0.05f, 0.05f),
                                 chrono::ChColor(0.55f, 0.55f, 0.55f));
    }

    // Returns false once the window is closed
    bool show(const std::array<Follower *, 2> &followers, const Path &path,
              const std::array<real_t, 2> &errors)
    {
        if (!vis_.Run())
        {
            return false;
        }
        for (uint_t i = 0; i < 2; ++i)
        {
            const Vector x = world_.pose(i);
            trails_[i].push_back({x[0] + World::offset(i), x[1], 0.012});
        }

        vis_.BeginScene();
        vis_.Render();
        const std::array<chrono::ChColor, 2> colours{chrono::ChColor(1.0f, 0.85f, 0.2f),
                                                     chrono::ChColor(0.3f, 0.7f, 1.0f)};
        for (uint_t i = 0; i < 2; ++i)
        {
            const real_t dx = World::offset(i);
            // the path, the robot's trail, its target and its plan
            for (std::size_t k = 1; k < figure_.size(); ++k)
            {
                bitsim::utils::render::irr_draw_line(
                    vis_, {figure_[k - 1][0] + dx, figure_[k - 1][1], 0.004},
                    {figure_[k][0] + dx, figure_[k][1], 0.004}, chrono::ChColor(0.8f, 0.8f, 0.8f),
                    2.0);
            }
            for (std::size_t k = 1; k < trails_[i].size(); ++k)
            {
                bitsim::utils::render::irr_draw_line(vis_, trails_[i][k - 1], trails_[i][k],
                                                     colours[i], 3.0);
            }
            const auto &plan = followers[i]->plan();
            for (std::size_t k = 1; k < plan.size(); ++k)
            {
                bitsim::utils::render::irr_draw_line(
                    vis_, {plan[k - 1][0] + dx, plan[k - 1][1], 0.02},
                    {plan[k][0] + dx, plan[k][1], 0.02}, chrono::ChColor(0.9f, 0.3f, 0.9f), 2.0);
            }
            const auto target = path.point(world_.time());
            bitsim::utils::render::irr_draw_line(vis_, {target[0] + dx, target[1], 0.0},
                                                 {target[0] + dx, target[1], 0.25},
                                                 chrono::ChColor(0.9f, 0.1f, 0.1f), 3.0);
        }

        char text[200];
        std::snprintf(text, sizeof(text),
                      "t %5.1f s   left: clipped LQR, %.2f m off the path   right: MPC, %.2f m "
                      "off the path   wheels at most %.1f m/s   magenta: the plans",
                      world_.time(), errors[0], errors[1], MAX_WHEEL_SPEED);
        bitsim::utils::render::draw_status_bar(vis_, text);
        vis_.EndScene();
        recorder_.record(vis_, world_.time());
        return true;
    }

  private:
    World &world_;
    chrono::irrlicht::ChVisualSystemIrrlicht vis_;
    bitsim::utils::render::FrameRecorder recorder_;
    std::vector<std::array<real_t, 2>> figure_;
    std::array<std::vector<chrono::ChVector3d>, 2> trails_;
};
#endif

struct Options
{
    bool render{false};
    std::string record;
    int laps{2};
};

int run(const Options &options)
{
    const Path path;
    World world;
    const real_t axle = world.robot(0).get_data().axle_length;

    // both start on the path
    const Vector start = path.state(0.0);
    for (uint_t i = 0; i < 2; ++i)
    {
        world.place(i, start[0], start[1], start[2]);
    }
    world.add_robots();

    Follower clipped(false, axle);
    Follower mpc(true, axle);
    const std::array<Follower *, 2> followers{&clipped, &mpc};
    const std::array<const char *, 2> names{"clipped LQR", "MPC"};

    real_t peak = 0.0;
    for (int k = 0; k < 1600; ++k)
    {
        const Vector u = path.wheels(LAP_TIME * k / 1600.0, axle);
        peak = std::max(peak, u.cwiseAbs().maxCoeff());
    }
    std::printf("Robot: %s, axle %.2f m; along the path a wheel rolls at %.2f m/s at most, the "
                "limit is %.1f m/s\n\n",
                ROBOT_FILE.c_str(), axle, peak, MAX_WHEEL_SPEED);

#ifdef BITSIM_IRRLICHT
    std::optional<Display> display;
    if (options.render || !options.record.empty())
    {
        display.emplace(world, path, options.record);
    }
#else
    if (options.render || !options.record.empty())
    {
        std::cerr << "--render and --record need bitsim built with Irrlicht\n";
        return 1;
    }
#endif

    std::printf("  time [s]   clipped LQR: off the path [m]   MPC: off the path [m]\n");
    std::array<real_t, 2> errors{0.0, 0.0};
    std::array<real_t, 2> sum_sq{0.0, 0.0};
    std::array<real_t, 2> worst_after{0.0, 0.0};
    uint_t n_samples = 0;
    uint_t unsolved = 0;
    std::array<Vector, 2> wheels{Vector::Zero(2), Vector::Zero(2)};
    real_t next_control = 0.0;
    real_t next_print = 0.0;
    real_t next_frame = 0.0;
    const auto wall_start = std::chrono::steady_clock::now();
    while (world.time() < options.laps * LAP_TIME - 0.5 * STEP_SIZE)
    {
        const real_t t = world.time();
        if (t >= next_control - 1e-9)
        {
            for (uint_t i = 0; i < 2; ++i)
            {
                const Vector x = world.pose(i);
                wheels[i] = followers[i]->control(path, t, world.state(i));
                world.set_wheels(i, wheels[i]);
                errors[i] = distance_to_path(path, x);
                sum_sq[i] += errors[i] * errors[i];
                if (t > SETTLE)
                {
                    worst_after[i] = std::max(worst_after[i], errors[i]);
                }
            }
            unsolved += mpc.mpc().status() == LinearMPC::QPStatus::SOLVED ? 0 : 1;
            ++n_samples;
            next_control += CONTROL_PERIOD;
        }
        if (t >= next_print - 1e-9)
        {
            std::printf("  %8.1f   %29.3f   %21.3f\n", t, errors[0], errors[1]);
            next_print += PRINT_PERIOD;
        }
        world.step();

#ifdef BITSIM_IRRLICHT
        if (display && world.time() >= next_frame - 1e-9)
        {
            next_frame += 1.0 / RENDER_RATE;
            if (!display->show(followers, path, errors))
            {
                return 0;
            }
            if (options.render)
            {
                const real_t ahead =
                    world.time() -
                    std::chrono::duration<real_t>(std::chrono::steady_clock::now() - wall_start)
                        .count();
                if (ahead > 0.0)
                {
                    std::this_thread::sleep_for(std::chrono::duration<real_t>(ahead));
                }
            }
        }
#else
        (void)next_frame;
        (void)wall_start;
#endif
    }

    std::printf("\n%-12s  RMS off the path [m]   worst after the first %.0f s [m]\n", "", SETTLE);
    for (uint_t i = 0; i < 2; ++i)
    {
        std::printf("%-12s  %20.3f   %30.3f\n", names[i], std::sqrt(sum_sq[i] / n_samples),
                    worst_after[i]);
    }
    std::printf("\nMPC solved %u QPs, %u of them not to the tolerance\n",
                static_cast<unsigned>(n_samples), static_cast<unsigned>(unsolved));

    // what the example shows: MPC stays on the path, closer than the clipped LQR
    return (worst_after[1] < 0.15 && worst_after[1] < worst_after[0]) ? 0 : 1;
}

} // namespace diff_drive_mpc

int main(int argc, char **argv)
{
    diff_drive_mpc::Options options;
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "--render")
        {
            options.render = true;
        }
        else if (arg == "--record" && i + 1 < argc)
        {
            options.record = argv[++i];
        }
        else if (arg == "--laps" && i + 1 < argc)
        {
            options.laps = std::max(1, std::atoi(argv[++i]));
        }
        else
        {
            std::cerr << "Usage: diff_drive_mpc [--render] [--record DIR] [--laps N]\n";
            return 1;
        }
    }
    return diff_drive_mpc::run(options);
}

#else
#include <iostream>
int main()
{
    std::cout << "This example needs bitsim built with Chrono\n";
    return 0;
}
#endif

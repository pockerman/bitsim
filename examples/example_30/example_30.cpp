#include "bitrl/bitrl_config.h"

#if defined(BITRL_CHRONO) && defined(BITRL_MQTT)

#include "bitrl/actuators/backends/mqtt_dc_motor_backend.h"
#include "bitrl/actuators/backends/sim_dc_motor_backend.h"
#include "bitrl/actuators/dc_motor.h"
#include "bitrl/bitrl_consts.h"
#include "bitrl/bitrl_types.h"
#include "bitrl/boards/arduino/dc_arduino_board_impl.h"
#include "bitrl/extern/nlohmann/json/json.hpp"
#include "bitrl/network/mqtt_publisher.h"
#include "bitrl/network/mqtt_subscriber.h"
#include "bitrl/rigid_bodies/chrono_robots/diff_drive_robot.h"

#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChContactMaterialNSC.h>
#include <chrono/physics/ChSystemNSC.h>

#ifdef BITRL_IRRLICHT
#include "bitrl/utils/render/irrlicht_utils.h"
#include <chrono_irrlicht/ChIrrTools.h>
#include <chrono_irrlicht/ChVisualSystemIrrlicht.h>
#endif

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace example_30
{

using bitrl::real_t;
using bitrl::uint_t;
using bitrl::actuators::DCMotor;
using bitrl::boards::arduino::DCArduinoBoardImpl;
using bitrl::rb::bitrl_chrono::CHRONO_DiffDriveRobot;
using chrono::ChVector3d;
using Clock = std::chrono::steady_clock;

const real_t PI = bitrl::consts::maths::PI;
const real_t DEG = PI / 180.0;
// Chrono steps every 10 ms. That is also how often the motor controller runs
const real_t STEP_SIZE = 0.01;

const std::string ROBOT_FILE = bitrl::consts::ROBOTS_DIR + "/bitrl_diff_drive_robot.json";

// The topics of the board. The sketch uses the same
const std::string COMMAND_TOPIC = "bitrl/dc_board/command";
const std::string STATE_TOPIC = "bitrl/dc_board/state";

// The board and the simulation exchange messages at 50 Hz
const real_t NETWORK_PERIOD = 0.02;

// How quickly the wheels follow what the board reports [rad/s]. Faster means
// less delay, but more of the noise of the encoders reaches Chrono
const real_t OBSERVER_BANDWIDTH = 10.0;

real_t clean(real_t x) { return std::abs(x) < 5.0e-4 ? 0.0 : x; }

real_t wrap(real_t a) { return std::atan2(std::sin(a), std::cos(a)); }

real_t seconds_between(Clock::time_point from, Clock::time_point to)
{
    return std::chrono::duration<real_t>(to - from).count();
}

void sleep_until_step(Clock::time_point start, long step)
{
    std::this_thread::sleep_until(
        start + std::chrono::microseconds(static_cast<long>(1.0e6 * STEP_SIZE * step)));
}

// ---------------------------------------------------------------------------
// The plan: a square with sides of 1 m, driving forwards and turning on the spot
// ---------------------------------------------------------------------------
struct Segment
{
    std::string name;
    real_t v;        // [m/s]
    real_t omega;    // [rad/s]
    real_t duration; // [s]
};

std::vector<Segment> square_plan()
{
    const real_t speed = 0.25;
    const real_t turn_rate = 45.0 * DEG;
    std::vector<Segment> plan;
    for (int side = 0; side < 4; ++side)
    {
        plan.push_back({"forward", speed, 0.0, 1.0 / speed});
        plan.push_back({"stop", 0.0, 0.0, 1.0});
        plan.push_back({"turn left", 0.0, turn_rate, 90.0 * DEG / turn_rate});
        plan.push_back({"stop", 0.0, 0.0, 1.0});
    }
    return plan;
}

real_t plan_duration(const std::vector<Segment> &plan)
{
    real_t total = 0.0;
    for (const auto &segment : plan)
    {
        total += segment.duration;
    }
    return total;
}

const Segment &segment_at(const std::vector<Segment> &plan, real_t t, bool repeat)
{
    if (repeat)
    {
        t = std::fmod(t, plan_duration(plan));
    }
    for (const auto &segment : plan)
    {
        if (t < segment.duration)
        {
            return segment;
        }
        t -= segment.duration;
    }
    static const Segment done{"done", 0.0, 0.0, 0.0};
    return done;
}

// ---------------------------------------------------------------------------
// The robot file has the wheel radius and the constants of the motors. The
// fake board uses them too
// ---------------------------------------------------------------------------
nlohmann::json read_robot_file()
{
    std::ifstream file(ROBOT_FILE);
    return nlohmann::json::parse(file);
}

real_t robot_wheel_radius()
{
    return read_robot_file().at("robot-data").at("wheel_radius").get<real_t>();
}

bitrl::actuators::backends::DCMotorParameters robot_motor_parameters()
{
    bitrl::actuators::backends::DCMotorParameters parameters;
    parameters.read(read_robot_file().at("motors").at("parameters"));
    return parameters;
}

// ---------------------------------------------------------------------------
// The board side, in software. Two simulated motors that turn freely, as the
// real ones do with the wheels off the ground, behind the same messages as the
// sketch
// ---------------------------------------------------------------------------
void run_fake_board(const std::string &broker)
{
    using bitrl::actuators::backends::SIM_DCMotorBackend;

    bitrl::network::MqttSubscriber commands(broker, COMMAND_TOPIC);
    bitrl::network::MqttPublisher state(broker, STATE_TOPIC);
    commands.connect();
    state.connect();
    std::cout << "Fake board connected to " << broker << ". Press Ctrl+C to stop\n";

    const real_t wheel_radius = robot_wheel_radius();
    const auto parameters = robot_motor_parameters();
    std::array<SIM_DCMotorBackend, 2> motors{SIM_DCMotorBackend(parameters),
                                             SIM_DCMotorBackend(parameters)};
    const std::array<std::string, 2> names{"left", "right"};
    long command_seq = -1;
    auto last_command = Clock::now();

    // The motors run at 1 kHz and the board talks to the network every 20 ms
    const real_t dt = 0.001;
    const int steps_per_message = static_cast<int>(std::round(NETWORK_PERIOD / dt));
    const auto start = Clock::now();

    for (long n = 0;; ++n)
    {
        for (auto &motor : motors)
        {
            motor.step(dt);
        }

        if (n % steps_per_message == 0)
        {
            while (const auto text = commands.poll(std::chrono::milliseconds(0)))
            {
                const auto message = nlohmann::json::parse(*text, nullptr, false);
                if (message.is_discarded() || !message.contains("seq") ||
                    !message.contains("left_speed") || !message.contains("right_speed"))
                {
                    continue;
                }
                command_seq = message["seq"].get<long>();
                for (std::size_t m = 0; m < 2; ++m)
                {
                    motors[m].set_target_speed(message[names[m] + "_speed"].get<real_t>());
                }
                last_command = Clock::now();
            }

            // Like the sketch, stop when the commands stop
            if (seconds_between(last_command, Clock::now()) > 0.5)
            {
                for (auto &motor : motors)
                {
                    motor.set_target_speed(0.0);
                }
            }

            nlohmann::json message{{"ack", command_seq}};
            for (std::size_t m = 0; m < 2; ++m)
            {
                message[names[m] + "_distance"] = wheel_radius * motors[m].encoder_angle();
                message[names[m] + "_speed"] = motors[m].measured_speed();
            }
            message["t_ms"] =
                std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - start).count();
            state.publish(message);
        }

        if (n % 1000 == 0)
        {
            std::cout << "  t = " << std::fixed << std::setprecision(0) << n * dt
                      << " s  targets = " << std::setprecision(2) << motors[0].target_speed()
                      << ", " << motors[1].target_speed()
                      << " rad/s  distances = " << std::setprecision(3)
                      << wheel_radius * motors[0].encoder_angle() << ", "
                      << wheel_radius * motors[1].encoder_angle() << " m" << std::endl;
        }

        std::this_thread::sleep_until(start + std::chrono::microseconds(1000 * (n + 1)));
    }
}

// ---------------------------------------------------------------------------
// The simulation side
// ---------------------------------------------------------------------------

// Where the robot thinks it is, from the distances its wheels covered
class Odometry
{
  public:
    explicit Odometry(real_t axle_length) : axle_length_(axle_length) {}

    void update(real_t left_distance, real_t right_distance)
    {
        const real_t dl = left_distance - left_;
        const real_t dr = right_distance - right_;
        left_ = left_distance;
        right_ = right_distance;

        const real_t d = 0.5 * (dl + dr);
        const real_t dtheta = (dr - dl) / axle_length_;
        x_ += d * std::cos(theta_ + 0.5 * dtheta);
        y_ += d * std::sin(theta_ + 0.5 * dtheta);
        theta_ += dtheta;
    }

    real_t x() const { return x_; }
    real_t y() const { return y_; }
    real_t theta() const { return theta_; }

  private:
    real_t axle_length_;
    real_t left_ = 0.0;
    real_t right_ = 0.0;
    real_t x_ = 0.0;
    real_t y_ = 0.0;
    real_t theta_ = 0.0;
};

// The robot of example_14 on a floor, with its motors simulated or on a board
class World
{
  public:
    explicit World(std::shared_ptr<DCArduinoBoardImpl> board) : board_(board)
    {
        system_.SetGravitationalAcceleration(ChVector3d(0.0, 0.0, -bitrl::consts::maths::G));
        system_.SetCollisionSystemType(chrono::ChCollisionSystem::Type::BULLET);
        // The robot runs in real time when its motors are real, so the solver
        // has to be quick
        system_.SetSolverType(chrono::ChSolver::Type::BARZILAIBORWEIN);
        system_.GetSolver()->AsIterative()->SetMaxIterations(40);

        auto material = chrono_types::make_shared<chrono::ChContactMaterialNSC>();
        material->SetFriction(0.8f);
        material->SetRestitution(0.1f);
        // The floor is only drawn as a grid
        auto floor = chrono_types::make_shared<chrono::ChBodyEasyBox>(5.0, 5.0, 0.1, 1000.0, false,
                                                                      true, material);
        floor->SetPos(ChVector3d(0.5, 0.5, -0.05));
        floor->SetFixed(true);
        system_.Add(floor);

        robot_.load(std::filesystem::path(ROBOT_FILE));

        // Real motors replace the simulated ones of the robot file
        if (board_)
        {
            for (uint_t m = 0; m < 2; ++m)
            {
                auto backend = std::make_shared<bitrl::actuators::backends::MQTT_DCMotorBackend>(
                    board_, m, OBSERVER_BANDWIDTH);
                robot_.set_motor(m, std::make_shared<DCMotor>(backend, m == 0 ? "left" : "right"));
            }
        }
        robot_.add_to_sys(system_);
        robot_.get_chassis()->GetVisualShape(0)->SetColor(chrono::ChColor(0.55f, 0.70f, 0.90f));

        odometry_ = Odometry(robot_.get_data().axle_length);
        start_x_ = robot_.axle_center().x();
        start_y_ = robot_.axle_center().y();
    }

    void step(const Segment &segment)
    {
        robot_.set_velocity(segment.v, segment.omega);
        robot_.update(STEP_SIZE);
        system_.DoStepDynamics(STEP_SIZE);

        odometry_.update(distance(0), distance(1));
        for (uint_t m = 0; m < 2; ++m)
        {
            max_wheel_torque_ = std::max(max_wheel_torque_, std::abs(robot_.wheel_torque(m)));
        }
    }

    real_t time() const { return system_.GetChTime(); }

    // The pose of the robot as Chrono has it, from where it started. The
    // position is the middle of the axle, the point the odometry tracks
    real_t x() const { return robot_.axle_center().x() - start_x_; }
    real_t y() const { return robot_.axle_center().y() - start_y_; }
    real_t theta() const
    {
        const auto forward = robot_.get_pose().rotation_matrix() * ChVector3d(1.0, 0.0, 0.0);
        return std::atan2(forward.y(), forward.x());
    }

    // The distance a wheel covered, as its motor reports it
    real_t distance(uint_t m) const
    {
        return robot_.get_data().wheel_radius * robot_.get_motor(m)->angle();
    }

    bool motors_ready() const
    {
        return robot_.get_motor(0)->is_ready() && robot_.get_motor(1)->is_ready();
    }

    const Odometry &odometry() const { return odometry_; }
    const CHRONO_DiffDriveRobot &robot() const { return robot_; }
    chrono::ChSystemNSC &system() { return system_; }
    const DCArduinoBoardImpl *board() const { return board_.get(); }
    real_t max_wheel_torque() const { return max_wheel_torque_; }

  private:
    std::shared_ptr<DCArduinoBoardImpl> board_;
    chrono::ChSystemNSC system_;
    CHRONO_DiffDriveRobot robot_;
    Odometry odometry_{1.0};
    real_t start_x_ = 0.0;
    real_t start_y_ = 0.0;
    real_t max_wheel_torque_ = 0.0;
};

// Connects to the board and waits for its first report
std::shared_ptr<DCArduinoBoardImpl> connect_board(const std::string &broker)
{
    DCArduinoBoardImpl::Config config;
    config.broker = broker;
    config.command_topic = COMMAND_TOPIC;
    config.state_topic = STATE_TOPIC;
    config.wheel_radius = robot_wheel_radius();
    config.command_period = NETWORK_PERIOD;

    auto board = std::make_shared<DCArduinoBoardImpl>(config);
    board->connect();
    std::cout << "  on a board, broker " << broker << "\n"
              << "  commands to the board on   " << COMMAND_TOPIC << "\n"
              << "  reports from the board on  " << STATE_TOPIC << "\n"
              << "Waiting for the board ...\n";

    if (!board->wait_for_board(std::chrono::seconds(10)))
    {
        std::cerr << "No message from the board in 10 s. Is it connected to the same broker?\n"
                  << "Without hardware, run ./example_30 --fake-board in another terminal,\n"
                  << "or ./example_30 --simulated-motors to simulate the motors as well\n";
        return nullptr;
    }
    return board;
}

void print_row(const World &world, const Segment &segment)
{
    const auto &left = *world.robot().get_motor(0);
    const auto &right = *world.robot().get_motor(1);
    std::cout << "  " << std::fixed << std::setprecision(1) << std::setw(5) << world.time() << "  "
              << std::left << std::setw(10) << segment.name << std::right << std::setprecision(2)
              << std::setw(6) << left.target_speed() << std::setw(6) << right.target_speed()
              << std::setw(7) << clean(left.speed()) << std::setw(6) << clean(right.speed())
              << std::setprecision(3) << std::setw(8) << clean(world.distance(0)) << std::setw(7)
              << clean(world.distance(1)) << std::setprecision(2) << std::setw(8)
              << clean(world.x()) << std::setw(6) << clean(world.y()) << std::setprecision(0)
              << std::setw(6) << clean(world.theta() / DEG) << std::setprecision(2) << std::setw(8)
              << clean(world.odometry().x()) << std::setw(6) << clean(world.odometry().y())
              << std::setprecision(0) << std::setw(6)
              << clean(wrap(world.odometry().theta()) / DEG);
    if (world.board())
    {
        std::cout << std::setprecision(1) << std::setw(8) << 1000.0 * world.board()->round_trip()
                  << (world.motors_ready() ? "" : "  board silent");
    }
    std::cout << "\n";
}

void print_pose(const std::string &label, real_t x, real_t y, real_t theta)
{
    std::cout << "  " << std::left << std::setw(34) << label << std::right << std::fixed
              << std::setprecision(3) << std::setw(7) << clean(x) << std::setw(8) << clean(y)
              << std::setprecision(1) << std::setw(8) << clean(theta / DEG) << " deg\n";
}

void run(std::shared_ptr<DCArduinoBoardImpl> board)
{
    World world(board);
    const auto plan = square_plan();
    const real_t duration = plan_duration(plan);

    std::cout << "\n2. Driving a square with sides of 1 m\n"
              << "                     target       speed      distance      robot (Chrono)"
                 "      odometry"
              << (board ? "      round" : "") << "\n"
              << "  t (s)  segment   (rad/s)      (rad/s)       (m)        x     y    th"
                 "      x     y    th"
              << (board ? "   trip (ms)" : "") << "\n";

    // With a board the motors are real, so the simulation runs in real time
    const auto start = Clock::now();
    long step = 0;
    const real_t rest = 1.0;
    while (world.time() < duration + rest - STEP_SIZE / 2.0)
    {
        const auto &segment = segment_at(plan, world.time(), false);
        world.step(segment);
        ++step;

        if (step % static_cast<long>(std::round(1.0 / STEP_SIZE)) == 0 &&
            world.time() < duration + STEP_SIZE)
        {
            print_row(world, segment);
        }
        if (board)
        {
            sleep_until_step(start, step);
        }
    }

    std::cout << "\n3. Summary\n"
              << std::fixed << std::setprecision(3) << "  wheel distances left / right    "
              << world.distance(0) << " / " << world.distance(1) << " m\n"
              << "                                          x       y   heading\n";
    print_pose("end of the square, planned", 0.0, 0.0, 0.0);
    print_pose("end of the square, Chrono", world.x(), world.y(), world.theta());
    print_pose("end of the square, odometry", world.odometry().x(), world.odometry().y(),
               wrap(world.odometry().theta()));

    const real_t drift =
        std::hypot(world.x() - world.odometry().x(), world.y() - world.odometry().y());
    std::cout << std::setprecision(3) << "  odometry error                  " << drift << " m, "
              << std::setprecision(1)
              << std::abs(wrap(world.theta() - world.odometry().theta())) / DEG << " deg\n";

    if (board)
    {
        std::cout << "  messages sent / received        " << board->n_sent() << " / "
                  << board->n_received() << "\n"
                  << std::setprecision(1) << "  round trip mean / max           "
                  << 1000.0 * board->mean_round_trip() << " / " << 1000.0 * board->max_round_trip()
                  << " ms\n";
    }

    std::cout << std::setprecision(3) << "  largest wheel torque            "
              << world.max_wheel_torque() << " N m\n"
              << "  stall torque of a motor at 12 V " << robot_motor_parameters().stall_torque()
              << " N m\n";
}

#ifdef BITRL_IRRLICHT
// A line through the points
void draw_path(chrono::irrlicht::ChVisualSystemIrrlicht &vis, const std::vector<ChVector3d> &path,
               const chrono::ChColor &color, real_t width)
{
    for (std::size_t i = 1; i < path.size(); ++i)
    {
        bitrl::utils::render::irr_draw_line(vis, path[i - 1], path[i], color, width);
    }
}

void render(std::shared_ptr<DCArduinoBoardImpl> board)
{
    using namespace bitrl::utils::render;

    World world(board);
    const auto plan = square_plan();

    chrono::irrlicht::ChVisualSystemIrrlicht vis;
    vis.AttachSystem(&world.system());
    vis.SetCameraVertical(chrono::CameraVerticalDir::Z);
    vis.SetWindowSize(1280, 720);
    vis.SetWindowTitle("Example 30: A differential drive robot with real motors");
    vis.Initialize();
    vis.AddCamera({0.5, -1.0, 1.5}, {0.5, 0.45, 0.0});
    vis.AddTypicalLights();
    vis.BindAll();

    auto *readout = vis.GetGUIEnvironment()->addStaticText(
        L"", irr::core::rect<irr::s32>(1010, 10, 1270, 215), true, true);
    readout->setBackgroundColor(irr::video::SColor(220, 255, 255, 255));
    readout->setDrawBackground(true);

    const std::vector<ChVector3d> planned{{0.0, 0.0, 0.005},
                                          {1.0, 0.0, 0.005},
                                          {1.0, 1.0, 0.005},
                                          {0.0, 1.0, 0.005},
                                          {0.0, 0.0, 0.005}};
    std::vector<ChVector3d> chrono_path;
    std::vector<ChVector3d> odometry_path;

    const int steps_per_frame = static_cast<int>(std::round(0.02 / STEP_SIZE));
    const auto start = Clock::now();
    long step = 0;

    while (vis.Run())
    {
        const auto &segment = segment_at(plan, world.time(), true);
        const auto &left = *world.robot().get_motor(0);
        const auto &right = *world.robot().get_motor(1);

        std::ostringstream text;
        text << std::fixed << std::setprecision(1) << "t = " << world.time() << " s\n"
             << (board ? (world.motors_ready() ? "motors on the board" : "BOARD SILENT")
                       : "simulated motors")
             << "\n";
        if (board)
        {
            text << "round trip = " << 1000.0 * board->round_trip() << " ms\n";
        }
        text << "\nsegment: " << segment.name << "\n"
             << std::setprecision(2) << "target    " << left.target_speed() << "  "
             << right.target_speed() << " rad/s\n"
             << "speed     " << clean(left.speed()) << "  " << clean(right.speed()) << " rad/s\n"
             << std::setprecision(3) << "distance  " << clean(world.distance(0)) << "  "
             << clean(world.distance(1)) << " m\n\n"
             << std::setprecision(2) << "Chrono    x " << clean(world.x()) << "  y "
             << clean(world.y()) << "  th " << std::setprecision(0) << clean(world.theta() / DEG)
             << "\n"
             << std::setprecision(2) << "odometry  x " << clean(world.odometry().x()) << "  y "
             << clean(world.odometry().y()) << "  th " << std::setprecision(0)
             << clean(wrap(world.odometry().theta()) / DEG) << "\n\n"
             << "white: the plan\nyellow: where the robot went\nred: where it thinks it went";
        readout->setText(irr::core::stringw(text.str().c_str()).c_str());

        vis.BeginScene();
        vis.Render();
        chrono::irrlicht::tools::drawGrid(&vis, 0.25, 0.25, 12, 12,
                                          chrono::ChCoordsysd(ChVector3d(0.5, 0.5, 0.0)),
                                          chrono::ChColor(0.35f, 0.45f, 0.55f));
        irr_draw_world_axes(vis, 0.3);
        draw_path(vis, planned, chrono::ChColor(1.0f, 1.0f, 1.0f), 1.0);
        draw_path(vis, chrono_path, chrono::ChColor(1.0f, 0.8f, 0.2f), 6.0);
        draw_path(vis, odometry_path, chrono::ChColor(0.9f, 0.2f, 0.2f), 2.0);

        for (int i = 0; i < steps_per_frame; ++i)
        {
            world.step(segment_at(plan, world.time(), true));
            ++step;
        }

        chrono_path.emplace_back(world.x(), world.y(), 0.01);
        odometry_path.emplace_back(world.odometry().x(), world.odometry().y(), 0.015);
        if (chrono_path.size() > 3000)
        {
            chrono_path.erase(chrono_path.begin());
            odometry_path.erase(odometry_path.begin());
        }

        vis.EndScene();
        sleep_until_step(start, step);
    }
}
#endif

} // namespace example_30

int main(int argc, char **argv)
{
    using namespace example_30;

    // ./example_30 [--simulated-motors | --fake-board] [--render] [broker]
    bool simulated_motors = false;
    bool fake_board = false;
    bool show = false;
    std::string broker = "tcp://localhost:1883";
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "--simulated-motors")
        {
            simulated_motors = true;
        }
        else if (arg == "--fake-board")
        {
            fake_board = true;
        }
        else if (arg == "--render")
        {
            show = true;
        }
        else if (arg.rfind("--", 0) == 0)
        {
            std::cerr << "Unknown option " << arg << "\n"
                      << "Usage: ./example_30 [--simulated-motors | --fake-board] [--render] "
                         "[broker]"
                      << std::endl;
            return 1;
        }
        else
        {
            broker = arg;
        }
    }

    try
    {
        if (fake_board)
        {
            run_fake_board(broker);
            return 0;
        }

        std::shared_ptr<DCArduinoBoardImpl> board;
        std::cout << "\n1. The motors\n";
        if (simulated_motors)
        {
            std::cout << "  simulated, with the constants of " << ROBOT_FILE << "\n";
        }
        else
        {
            board = connect_board(broker);
            if (!board)
            {
                return 1;
            }
        }

        if (show)
        {
#ifdef BITRL_IRRLICHT
            render(board);
#else
            std::cerr << "--render needs bitrl configured with BITRL_IRRLICHT=ON" << std::endl;
            return 1;
#endif
        }
        else
        {
            run(board);
        }
    }
    catch (const std::exception &error)
    {
        std::cerr << "Error: " << error.what() << std::endl;
        if (!simulated_motors)
        {
            std::cerr << "Is the MQTT broker running at " << broker << "?" << std::endl;
        }
        return 1;
    }

    return 0;
}

#else
#include <iostream>
int main()
{
    std::cerr << "You need bitrl configured with PROJECTCHRONO and MQTT "
              << "in order to run this example. "
              << "Reconfigure bitrl and set ENABLE_CHRONO=ON and ENABLE_MQTT=ON" << std::endl;
    return 1;
}
#endif

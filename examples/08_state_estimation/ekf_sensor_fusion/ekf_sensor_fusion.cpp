// Sensor fusion with an extended Kalman filter
//
// The wheel_odometry example again, in a room: the differential drive robot follows a line with
// its infrared elements and drives around the boxes on it, turning by its gyroscope and measuring
// by its encoders. Alongside, an extended Kalman filter, bitsim's DiffDriveEKF, works out where the
// robot is from everything it senses:
//
// - the encoders and the gyroscope predict where it moved, and the filter learns the gyroscope's
//   bias as it goes;
// - the infrared elements and the camera see the line, which the robot knows is the track;
// - the front and the rear ultrasonic sensors range the walls of the room and the boxes, which
//   the robot knows too.
//
// At the end the example says how far the filter and odometry alone drifted from the true pose.
//
// Usage: ekf_sensor_fusion [--render] [--record DIR] [--radius-error PERCENT] [--no-calibration]
//                          [--no-ir] [--no-camera] [--no-sonar]
//
// --render shows the simulation in a window, in real time, with the true path, the path of
// odometry and the path of the filter. --record DIR also writes the window to DIR, ten pictures a
// second. --radius-error makes odometry and the filter take the wheels to be PERCENT bigger than
// they are, which makes every distance PERCENT too long. --no-calibration skips odometry's
// measuring of the gyro's bias; the filter learns it anyway. --no-ir, --no-camera and --no-sonar
// keep that sensor out of the filter.

#include "bitsim/bitsim_consts.h"
#include "bitsim/bitsim_types.h"
#include "bitsim/control/infrared_line_follower.h"
#include "bitsim/estimation/diff_drive_ekf.h"
#include "bitsim/estimation/diff_drive_odometry.h"
#include "bitsim/estimation/gyro_heading.h"
#include "bitsim/rigid_bodies/chrono_robots/contact_counter.h"
#include "bitsim/rigid_bodies/chrono_robots/diff_drive_robot.h"
#include "bitsim/sensors/backends/chrono_imu_backend.h"
#include "bitsim/sensors/backends/chrono_ultrasound_backend.h"
#include "bitsim/sensors/downward_camera.h"
#include "bitsim/sensors/imu_sensor.h"
#include "bitsim/sensors/infrared_sensor.h"
#include "bitsim/sensors/ultrasonic_sensor.h"
#include "bitsim/trajectory/rounded_rectangle.h"
#include "bitsim/utils/render/irrlicht_scene.h"
#include "bitsim/utils/render/irrlicht_utils.h"

#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChContactMaterialNSC.h>
#include <chrono/physics/ChSystemNSC.h>
#include <chrono_irrlicht/ChVisualSystemIrrlicht.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace ekf_sensor_fusion
{

using bitsim::real_t;
using bitsim::uint_t;
using bitsim::control::InfraredLineFollower;
using bitsim::estimation::Box2d;
using bitsim::estimation::DiffDriveEKF;
using bitsim::estimation::DiffDriveMap;
using bitsim::estimation::DiffDriveOdometry;
using bitsim::estimation::GyroHeading;
using bitsim::rb::bitsim_chrono::CHRONO_DiffDriveRobot;
using bitsim::rb::bitsim_chrono::ContactCounter;
using bitsim::sensors::FloorImage;
using bitsim::sensors::IMUSensor;
using bitsim::sensors::InfraredSensor;
using bitsim::sensors::UltrasonicSensor;
using bitsim::trajectory::RoundedRectangle;
using bitsim::utils::render::FrameRecorder;
using bitsim::utils::render::InsetCamera;
using bitsim::utils::render::Trail;
using Command = bitsim::control::DiffDriveCommand;
using Point = std::array<real_t, 2>;

const std::string ROBOT_FILE = bitsim::consts::ROBOTS_DIR + "/bitsim_diff_drive_robot.json";
const std::string WINDOW_TITLE = "Sensor fusion with an extended Kalman filter";

const real_t PI = bitsim::consts::maths::PI;
const real_t STEP_SIZE = 0.01;   // [s]
const real_t TIME_LIMIT = 150.0; // [s]
const real_t PRINT_PERIOD = 1.0; // [s]
const real_t RENDER_RATE = 30.0; // [frames/s] of simulation in the window
const real_t RECORD_RATE = 10.0; // [frames/s] of simulation written by --record

// The track: a rounded rectangle, centred on the origin, drawn on paper
const real_t TRACK_HALF_LENGTH = 2.2;   // [m] along x, to the centre of the line
const real_t TRACK_HALF_WIDTH = 1.4;    // [m] along y
const real_t TRACK_CORNER_RADIUS = 0.7; // [m]
const real_t LINE_WIDTH = 0.05;         // [m]
const real_t FLOOR_WIDTH = 5.6;         // [m] the paper, along x, with room for detours
const real_t FLOOR_HEIGHT = 4.6;        // [m] along y
const real_t METRES_PER_PIXEL = 0.005;  // of the picture of the paper
const uchar PAPER = 235;                // the brightness of the paper
const uchar INK = 25;                   // and of the line

// The boxes on the track: where their centres are, and their size
const std::vector<Point> OBSTACLES{{0.0, -TRACK_HALF_WIDTH}, {-0.2, TRACK_HALF_WIDTH}};
const real_t OBSTACLE_SIZE = 0.3;   // [m] each side of its base
const real_t OBSTACLE_HEIGHT = 0.4; // [m]

// The room around the track: walls the ultrasonic sensors can hear
const real_t ROOM_HALF_LENGTH = 3.0; // [m] along x, to the inside of the walls
const real_t ROOM_HALF_WIDTH = 2.5;  // [m] along y
const real_t WALL_THICKNESS = 0.1;   // [m]
const real_t WALL_HEIGHT = 0.5;      // [m]

// What the filter takes each sensor's errors to be
const real_t IR_SIGMA = 0.02;     // [m] of where the line is under the elements, 4 cm apart
const real_t CAMERA_SIGMA = 0.01; // [m] of a point of the line in the camera's image

// The robot starts on the bottom side of the track and goes counterclockwise
const Point START{-1.3, -TRACK_HALF_WIDTH};

// Following the line
const real_t CRUISE_SPEED = 0.3;     // [m/s]
const real_t LINE_TURN_RATE = 1.6;   // [rad/s] with only an outer element on the line
const real_t SEARCH_TURN_RATE = 0.8; // [rad/s] while looking for the line
const real_t LOST_TIME = 3.0;        // [s] without the line before giving up

// Going around an obstacle
const real_t STOP_DISTANCE = 0.5;               // [m] closer than this, go around
const real_t TURN_AWAY_ANGLE = 60.0 * PI / 180; // [rad] to the right, before the detour
const real_t DETOUR_RADIUS = 1.2;               // [m] of the arc back to the line
const real_t DETOUR_SPEED = 0.25;               // [m/s]
const real_t PIVOT_RATE = 1.0;                  // [rad/s] turning on the spot

// Working out the heading
const real_t CALIBRATION_TIME = 1.0; // [s] standing still to measure the gyro's bias
const real_t TRAIL_STEP = 0.02;      // [m] between the points of the paths in the window

// The camera that looks ahead: where it sits on the chassis, and how it looks
const chrono::ChVector3d CAMERA_POSITION{0.26, 0.0, 0.07}; // from the centre of the chassis
const real_t CAMERA_TILT = 8.0 * PI / 180;                 // [rad] down from the horizon
const real_t CAMERA_FOV = 60.0 * PI / 180;                 // [rad] vertical
const int CAMERA_WIDTH = 384;                              // [pixels] in the window
const int CAMERA_HEIGHT = 216;

// ---------------------------------------------------------------------------
// The track
// ---------------------------------------------------------------------------

// The line on the paper
const RoundedRectangle TRACK(TRACK_HALF_LENGTH, TRACK_HALF_WIDTH, TRACK_CORNER_RADIUS);

// What the robot knows of its world: the track, the walls and the boxes
DiffDriveMap make_map()
{
    const real_t L = ROOM_HALF_LENGTH;
    const real_t W = ROOM_HALF_WIDTH;
    const real_t t = WALL_THICKNESS;
    std::vector<Box2d> obstacles{{-L - t, -W - t, -L, W + t},
                                 {L, -W - t, L + t, W + t},
                                 {-L, -W - t, L, -W},
                                 {-L, W, L, W + t}};
    for (const auto &b : OBSTACLES)
    {
        const real_t h = 0.5 * OBSTACLE_SIZE;
        obstacles.push_back({b[0] - h, b[1] - h, b[0] + h, b[1] + h});
    }
    return DiffDriveMap(TRACK, obstacles);
}

// The paper, with the line on it. It is both what the infrared elements look
// at and the texture of the floor in the window
FloorImage draw_floor()
{
    return FloorImage::draw(
        FLOOR_WIDTH, FLOOR_HEIGHT, METRES_PER_PIXEL,
        [](real_t x, real_t y) { return TRACK.distance(x, y) < 0.5 * LINE_WIDTH; }, PAPER, INK);
}

// ---------------------------------------------------------------------------
// Following the line
// ---------------------------------------------------------------------------

// The robot follows the line with bitsim's InfraredLineFollower: the three
// elements, left to right, say where the line is, and the robot turns towards
// it, harder the further out it is. With no element on the line it turns on the
// spot towards where it saw the line last
const InfraredLineFollower::Parameters FOLLOW_LINE{CRUISE_SPEED, LINE_TURN_RATE, SEARCH_TURN_RATE,
                                                   LOST_TIME};

// ---------------------------------------------------------------------------
// Going around obstacles
// ---------------------------------------------------------------------------

// The simplest detour that works: when the sonar hears a box closer than
// STOP_DISTANCE, turn right on the spot by TURN_AWAY_ANGLE, then drive an arc
// to the left. The arc swings out around the box and comes back to the line
// on the other side of it. When the middle element finds the line, drive on
// until the wheels are over it, turn right on the spot until the middle
// element is on it again, and follow the line.
//
// The track goes counterclockwise, so the outside of it, where there is room,
// is always on the right of the robot.
class Driver
{
  public:
    enum class State
    {
        FOLLOW,    // follow the line
        TURN_AWAY, // turn right on the spot, away from the box
        DETOUR,    // drive an arc to the left, around the box
        CROSS,     // drive on until the wheels are over the line
        ALIGN      // turn right on the spot onto the line
    };

    // ir_ahead is how far ahead of the axle the infrared elements are
    explicit Driver(real_t ir_ahead) : ir_ahead_(ir_ahead) {}

    // driven is how far the robot has driven, from its encoders
    Command command(const InfraredSensor &ir, real_t sonar, real_t driven, real_t heading,
                    real_t dt)
    {
        switch (state_)
        {
        case State::FOLLOW:
            if (sonar < STOP_DISTANCE)
            {
                turn_to_ = heading - TURN_AWAY_ANGLE;
                ++n_detours_;
                state_ = State::TURN_AWAY;
                return command(ir, sonar, driven, heading, dt);
            }
            return follower_.command(ir, dt);

        case State::TURN_AWAY:
            if (turn_towards_(heading))
            {
                return {0.0, -pivot_rate_(heading)};
            }
            state_ = State::DETOUR;
            [[fallthrough]];

        case State::DETOUR:
            if (ir.is_dark(middle_(ir)))
            {
                cross_from_ = driven;
                state_ = State::CROSS;
                return command(ir, sonar, driven, heading, dt);
            }
            return {DETOUR_SPEED, DETOUR_SPEED / DETOUR_RADIUS};

        case State::CROSS:
            // Until the wheels are over the line, by the encoders
            if (driven - cross_from_ < ir_ahead_)
            {
                return {DETOUR_SPEED, 0.0};
            }
            state_ = State::ALIGN;
            [[fallthrough]];

        case State::ALIGN:
            // Turning right, the elements sweep across the line: first the
            // right one, then the middle one
            if (!ir.is_dark(middle_(ir)) || ir.is_dark(ir.n_elements() - 1))
            {
                return {0.0, -0.5 * PIVOT_RATE};
            }
            state_ = State::FOLLOW;
            return follower_.command(ir, dt);
        }
        return {};
    }

    State state() const noexcept { return state_; }
    bool has_given_up() const { return follower_.has_given_up(); }
    bool is_lost() const { return state_ == State::FOLLOW && follower_.is_lost(); }
    uint_t n_detours() const noexcept { return n_detours_; }

  private:
    static uint_t middle_(const InfraredSensor &ir) { return ir.n_elements() / 2; }

    // True while the heading is not yet turn_to_
    bool turn_towards_(real_t heading) const
    {
        return std::abs(std::remainder(turn_to_ - heading, 2.0 * PI)) > 0.02;
    }

    // Slow down near the heading: the motors take a moment to stop
    real_t pivot_rate_(real_t heading) const
    {
        const real_t error = std::abs(std::remainder(turn_to_ - heading, 2.0 * PI));
        return std::clamp(2.0 * error, 0.2, PIVOT_RATE);
    }

    real_t ir_ahead_;
    InfraredLineFollower follower_{FOLLOW_LINE};
    State state_{State::FOLLOW};
    real_t turn_to_{0.0};
    real_t cross_from_{0.0};
    uint_t n_detours_{0};
};

std::string to_string(Driver::State state)
{
    switch (state)
    {
    case Driver::State::FOLLOW:
        return "follow";
    case Driver::State::TURN_AWAY:
        return "turn away";
    case Driver::State::DETOUR:
        return "detour";
    case Driver::State::CROSS:
        return "cross";
    case Driver::State::ALIGN:
        return "align";
    }
    return "";
}

// ---------------------------------------------------------------------------
// The heading from the gyroscope
// ---------------------------------------------------------------------------

// The robot's own idea of which way it faces. The gyroscope measures how fast
// the robot turns about the vertical, the z axis of the chassis; adding up
// rate x dt from a known start gives the heading:
//
//     heading(t + dt) = heading(t) + (gyro_z - bias) dt
//
// An error in the rate never goes away: it adds up, and the heading drifts.
// To keep the drift small the robot first stands still for CALIBRATION_TIME.
// It is not turning, so the mean of what the gyroscope reads is its bias.
// bitsim's GyroHeading does both

// ---------------------------------------------------------------------------
// The simulated world
// ---------------------------------------------------------------------------

class World
{
  public:
    explicit World(const FloorImage &floor)
    {
        system_.SetGravitationalAcceleration(
            chrono::ChVector3d(0.0, 0.0, -bitsim::consts::maths::G));
        system_.SetCollisionSystemType(chrono::ChCollisionSystem::Type::BULLET);
        system_.SetSolverType(chrono::ChSolver::Type::BARZILAIBORWEIN);
        system_.GetSolver()->AsIterative()->SetMaxIterations(40);

        auto material = chrono_types::make_shared<chrono::ChContactMaterialNSC>();
        material->SetFriction(0.8f);
        material->SetRestitution(0.1f);
        ground_ = chrono_types::make_shared<chrono::ChBodyEasyBox>(20.0, 20.0, 0.1, 1000.0, true,
                                                                   true, material);
        ground_->SetPos(chrono::ChVector3d(0.0, 0.0, -0.05));
        ground_->SetFixed(true);
        ground_->GetVisualShape(0)->SetColor(chrono::ChColor(0.45f, 0.45f, 0.42f));
        system_.Add(ground_);

        for (const auto &p : OBSTACLES)
        {
            auto box = chrono_types::make_shared<chrono::ChBodyEasyBox>(
                OBSTACLE_SIZE, OBSTACLE_SIZE, OBSTACLE_HEIGHT, 500.0, true, true, material);
            box->SetPos(chrono::ChVector3d(p[0], p[1], 0.5 * OBSTACLE_HEIGHT));
            box->SetFixed(true);
            box->GetVisualShape(0)->SetColor(chrono::ChColor(0.85f, 0.45f, 0.1f));
            system_.Add(box);
            obstacles_.push_back(box);
        }
        // The walls of the room. The robot must not touch them either
        const real_t L = ROOM_HALF_LENGTH + 0.5 * WALL_THICKNESS;
        const real_t W = ROOM_HALF_WIDTH + 0.5 * WALL_THICKNESS;
        const real_t long_side = 2.0 * (ROOM_HALF_LENGTH + WALL_THICKNESS);
        const real_t short_side = 2.0 * ROOM_HALF_WIDTH;
        for (const auto &[x, y, sx, sy] :
             std::vector<std::array<real_t, 4>>{{0.0, -W, long_side, WALL_THICKNESS},
                                                {0.0, W, long_side, WALL_THICKNESS},
                                                {-L, 0.0, WALL_THICKNESS, short_side},
                                                {L, 0.0, WALL_THICKNESS, short_side}})
        {
            auto wall = chrono_types::make_shared<chrono::ChBodyEasyBox>(sx, sy, WALL_HEIGHT, 500.0,
                                                                         true, true, material);
            wall->SetPos(chrono::ChVector3d(x, y, 0.5 * WALL_HEIGHT));
            wall->SetFixed(true);
            wall->GetVisualShape(0)->SetColor(chrono::ChColor(0.6f, 0.6f, 0.65f));
            system_.Add(wall);
            obstacles_.push_back(wall);
        }
        contacts_ = std::make_shared<ContactCounter>(obstacles_);

        robot_.load(std::filesystem::path(ROBOT_FILE));
        if (!robot_.get_line_sensor() || !robot_.get_sonar() || !robot_.get_rear_sonar() ||
            !robot_.get_imu() || !robot_.get_camera())
        {
            throw std::runtime_error("The robot file " + ROBOT_FILE +
                                     " needs a line-sensor, a front-sonar, a rear-sonar, an imu "
                                     "and a front-camera");
        }
        robot_.set_floor([&floor](real_t x, real_t y) { return floor.reflectance(x, y); });
        robot_.set_world_pose(START[0], START[1], 0.0);
        robot_.add_to_sys(system_);
    }

    void step(const Command &command)
    {
        robot_.set_velocity(command.v, command.omega);
        robot_.update(STEP_SIZE);
        system_.DoStepDynamics(STEP_SIZE);

        touching_ = contacts_->count(system_) > 0;
    }

    // Put the paper on the ground, so that the window shows the line: a
    // rectangle just above the ground with the picture on it. The top left
    // corner of the picture is the corner of the floor with the smallest x and
    // the largest y
    void show_floor(const std::string &texture)
    {
        // just above the centre of the ground, which is 0.1 thick
        bitsim::utils::render::add_floor_texture(*ground_, FLOOR_WIDTH, FLOOR_HEIGHT, 0.051,
                                                 texture);
    }

    real_t time() const { return system_.GetChTime(); }
    Point position() const { return {robot_.axle_center().x(), robot_.axle_center().y()}; }
    real_t heading() const { return robot_.heading(); }
    bool is_touching_an_obstacle() const { return touching_; }

    CHRONO_DiffDriveRobot &robot() { return robot_; }
    const CHRONO_DiffDriveRobot &robot() const { return robot_; }
    chrono::ChSystemNSC &system() { return system_; }

    // How far ahead of the axle centre the middle infrared element is
    real_t ir_ahead() const
    {
        const auto &elements = robot_.get_line_sensor()->infrared_backend()->elements();
        const auto &e = elements[elements.size() / 2];
        const auto chassis = robot_.get_chassis();
        const auto world = chassis->GetFrameRefToAbs().TransformPointLocalToParent(
            chrono::ChVector3d(e[0], e[1], e[2]));
        const auto ahead = world - robot_.axle_center();
        return ahead.x() * std::cos(heading()) + ahead.y() * std::sin(heading());
    }

    // A point of the chassis, relative to the centre of the axle: forward and to the left. The
    // sensors are placed on the chassis; the filter tracks the axle
    std::array<real_t, 2> from_axle(const chrono::ChVector3d &on_chassis) const
    {
        const auto world =
            robot_.get_chassis()->GetFrameRefToAbs().TransformPointLocalToParent(on_chassis);
        const auto d = world - robot_.axle_center();
        const real_t c = std::cos(heading());
        const real_t s = std::sin(heading());
        return {c * d.x() + s * d.y(), -s * d.x() + c * d.y()};
    }

    // An ultrasonic sensor as the filter sees it
    DiffDriveEKF::RangeSensor range_sensor(const UltrasonicSensor &sonar) const
    {
        const auto backend =
            std::dynamic_pointer_cast<bitsim::sensors::backends::CHRONO_UltrasonicBackend>(
                sonar.range_backend());
        if (!backend)
        {
            throw std::runtime_error("The sonars must have the SIMULATION backend");
        }
        const auto where = from_axle(backend->position());
        return {where[0],
                where[1],
                backend->yaw(),
                backend->beam_angle(),
                backend->n_rays(),
                backend->min_distance(),
                backend->max_distance()};
    }

  private:
    chrono::ChSystemNSC system_;
    std::shared_ptr<chrono::ChBodyEasyBox> ground_;
    std::vector<std::shared_ptr<chrono::ChBody>> obstacles_;
    std::shared_ptr<ContactCounter> contacts_;
    CHRONO_DiffDriveRobot robot_;
    bool touching_{false};
};

// ---------------------------------------------------------------------------
// The window
// ---------------------------------------------------------------------------

// The world from above the track, with what the robot's camera sees in the
// top left corner. The camera only shows; the robot does not use it
class Display
{
  public:
    Display(World &world, const FloorImage &floor, const std::string &record)
        : world_(world), recorder_(record, RECORD_RATE)
    {
        // The paper on the ground, as a texture. The visual system takes the
        // shapes of the bodies when it starts, so it goes on first.
        // Irrlicht reads colour pictures more reliably than grey ones
        texture_ = (std::filesystem::temp_directory_path() / "bitsim_ekf_sensor_fusion_floor.png")
                       .string();
        floor.write_texture(texture_);
        world_.show_floor(texture_);

        vis_.AttachSystem(&world_.system());
        vis_.SetWindowSize(1280, 720);
        vis_.SetWindowTitle(WINDOW_TITLE);
        vis_.SetCameraVertical(chrono::CameraVerticalDir::Z);
        vis_.Initialize();
        vis_.AddCamera({0.0, -3.4, 3.0}, {0.0, -0.1, 0.0});
        // A soft light: the typical lights wash the white paper out
        vis_.AddLightDirectional(60.0, 60.0, chrono::ChColor(0.45f, 0.45f, 0.45f),
                                 chrono::ChColor(0.05f, 0.05f, 0.05f),
                                 chrono::ChColor(0.55f, 0.55f, 0.55f));

        // The robot's camera, in the top left corner. The mouse moves the main
        // camera only
        robot_camera_.emplace(vis_, CAMERA_FOV, CAMERA_WIDTH, CAMERA_HEIGHT);
    }

    // Returns false once the window is closed
    bool show(real_t sonar, real_t rear_sonar, const DiffDriveOdometry &odometry,
              const DiffDriveEKF &ekf, const Driver &driver, real_t lap)
    {
        if (!vis_.Run())
        {
            return false;
        }

        vis_.BeginScene();
        vis_.Render();
        odometry_trail_.draw(vis_, chrono::ChColor(0.9f, 0.1f, 0.9f), 2.0);
        ekf_trail_.draw(vis_, chrono::ChColor(0.0f, 0.8f, 0.9f), 2.0);
        true_trail_.draw(vis_, chrono::ChColor(1.0f, 0.8f, 0.0f), 1.0);
        draw_sonar_(*world_.robot().get_sonar(), sonar);
        draw_sonar_(*world_.robot().get_rear_sonar(), rear_sonar);
        draw_robot_view_();
        draw_text_(odometry, ekf, driver, lap);
        vis_.EndScene();

        recorder_.record(vis_, world_.time());
        return true;
    }

    // Remember where the robot is, and where odometry and the filter think it is, for the paths
    // in the window
    void add_to_trails(const Point &truth, const Point &odometry, const Point &ekf)
    {
        true_trail_.add(truth[0], truth[1]);
        odometry_trail_.add(odometry[0], odometry[1]);
        ekf_trail_.add(ekf[0], ekf[1]);
    }

  private:
    // A sonar's beam, as far as the echo came from
    void draw_sonar_(const UltrasonicSensor &sensor, real_t range)
    {
        const auto &frame = world_.robot().get_chassis()->GetFrameRefToAbs();
        const auto backend =
            std::dynamic_pointer_cast<bitsim::sensors::backends::CHRONO_UltrasonicBackend>(
                sensor.range_backend());
        if (!backend)
        {
            return;
        }
        const chrono::ChVector3d start = frame.TransformPointLocalToParent(backend->position());
        const chrono::ChVector3d dir = frame.TransformDirectionLocalToParent(
            {std::cos(backend->yaw()), std::sin(backend->yaw()), 0.0});
        const auto colour = range < backend->max_distance() - 1e-6
                                ? chrono::ChColor(0.2f, 0.8f, 0.2f)
                                : chrono::ChColor(0.6f, 0.6f, 0.6f);
        bitsim::utils::render::irr_draw_line(vis_, start, start + dir * range, colour);
    }

    // Render the scene again, into the corner, from the robot's camera
    void draw_robot_view_()
    {
        const auto &frame = world_.robot().get_chassis()->GetFrameRefToAbs();
        const chrono::ChVector3d eye = frame.TransformPointLocalToParent(CAMERA_POSITION);
        const chrono::ChVector3d look = frame.TransformDirectionLocalToParent(
            {std::cos(CAMERA_TILT), 0.0, -std::sin(CAMERA_TILT)});
        const chrono::ChVector3d target = eye + look;
        robot_camera_->draw(eye, target);
    }

    void draw_text_(const DiffDriveOdometry &odometry, const DiffDriveEKF &ekf,
                    const Driver &driver, real_t lap)
    {
        const Point p = world_.position();
        const auto e = ekf.pose();
        std::ostringstream text;
        text << std::fixed << std::setprecision(1) << "t " << world_.time() << " s   lap "
             << 100.0 * lap << " %   off the true position by: odometry " << std::setprecision(3)
             << std::hypot(odometry.pose().x - p[0], odometry.pose().y - p[1])
             << " m (magenta), EKF " << std::hypot(e.x - p[0], e.y - p[1])
             << " m (cyan)   EKF heading off by " << std::setprecision(1)
             << std::remainder(e.yaw - world_.heading(), 2.0 * PI) * 180.0 / PI
             << " deg   gyro bias " << std::setprecision(4) << ekf.gyro_bias() << " rad/s   "
             << to_string(driver.state());
        bitsim::utils::render::draw_status_bar(vis_, text.str());
    }

    World &world_;
    FrameRecorder recorder_;
    std::string texture_;
    chrono::irrlicht::ChVisualSystemIrrlicht vis_;
    std::optional<InsetCamera> robot_camera_;
    Trail true_trail_{TRAIL_STEP};
    Trail odometry_trail_{TRAIL_STEP};
    Trail ekf_trail_{TRAIL_STEP};
};

// ---------------------------------------------------------------------------
// Running it
// ---------------------------------------------------------------------------

struct Options
{
    bool render{false};
    std::string record;
    real_t radius_error{0.0}; // [%] of the wheel radius that odometry and the filter assume
    bool calibrate{true};
    bool use_ir{true};
    bool use_camera{true};
    bool use_sonar{true};
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
        else if (arg == "--radius-error" && i + 1 < argc)
        {
            options.radius_error = std::stod(argv[++i]);
        }
        else if (arg == "--no-calibration")
        {
            options.calibrate = false;
        }
        else if (arg == "--no-ir")
        {
            options.use_ir = false;
        }
        else if (arg == "--no-camera")
        {
            options.use_camera = false;
        }
        else if (arg == "--no-sonar")
        {
            options.use_sonar = false;
        }
        else if (arg == "--record" && i + 1 < argc)
        {
            options.record = argv[++i];
            options.render = true;
        }
        else
        {
            std::cout
                << "Usage: ekf_sensor_fusion [--render] [--record DIR] [--radius-error PERCENT] "
                   "[--no-calibration] [--no-ir] [--no-camera] [--no-sonar]\n"
                << "  --render          show the simulation in a window, in real time\n"
                << "  --record DIR      also write the window to DIR, " << RECORD_RATE
                << " frames a second\n"
                << "  --radius-error P  odometry and the filter take the wheels to be P % "
                   "bigger\n"
                << "  --no-calibration  odometry does not measure the gyro's bias first\n"
                << "  --no-ir           keep the infrared elements out of the filter\n"
                << "  --no-camera       keep the camera out of the filter\n"
                << "  --no-sonar        keep the ultrasonic sensors out of the filter" << std::endl;
            std::exit(arg == "--help" || arg == "-h" ? 0 : 1);
        }
    }
    return options;
}

// How often each sensor's readings went into the filter, and how often the gate kept them out
struct Usage
{
    uint_t used{0};
    uint_t rejected{0};

    void count(const bitsim::estimation::EKFUpdateResult &result, bool tried)
    {
        if (!tried)
        {
            return;
        }
        (result.accepted ? used : rejected) += 1;
    }
};

// The largest and the mean distance of an estimate from the truth
struct ErrorStats
{
    real_t max{0.0};
    real_t sum{0.0};
    uint_t n{0};

    void add(real_t e)
    {
        max = std::max(max, e);
        sum += e;
        ++n;
    }
    real_t mean() const { return n ? sum / n : 0.0; }
};

int run(const Options &options)
{
    const FloorImage floor = draw_floor();
    World world(floor);
    auto &robot = world.robot();
    auto &ir = *robot.get_line_sensor();
    auto &sonar = *robot.get_sonar();
    auto &rear_sonar = *robot.get_rear_sonar();
    auto &imu = *robot.get_imu();
    const auto &camera = *robot.get_camera();

    std::optional<Display> display;
    if (options.render)
    {
        display.emplace(world, floor, options.record);
    }

    Driver driver(world.ir_ahead());
    GyroHeading heading(world.heading(), options.calibrate ? CALIBRATION_TIME : 0.0);

    // Odometry from the encoders with the heading from the gyroscope, as in wheel_odometry, and
    // the filter. Both take the wheels to be the radius that --radius-error says
    auto left = robot.get_motor(CHRONO_DiffDriveRobot::LEFT);
    auto right = robot.get_motor(CHRONO_DiffDriveRobot::RIGHT);
    const auto &data = robot.get_data();
    const real_t wheel_radius = data.wheel_radius * (1.0 + 0.01 * options.radius_error);
    DiffDriveOdometry odometry(wheel_radius, data.axle_length);
    odometry.reset({START[0], START[1], 0.0}, left->encoder_angle(), right->encoder_angle());

    DiffDriveEKF ekf(make_map(), {START[0], START[1], 0.0}, DiffDriveEKF::Parameters{});
    const auto front_range = world.range_sensor(sonar);
    const auto rear_range = world.range_sensor(rear_sonar);
    real_t last_left = left->encoder_angle();
    real_t last_right = right->encoder_angle();

    std::cout << std::fixed << std::setprecision(2) << "Track " << 2 * TRACK_HALF_LENGTH << " x "
              << 2 * TRACK_HALF_WIDTH << " m, " << OBSTACLES.size() << " boxes on it, in a room of "
              << 2 * ROOM_HALF_LENGTH << " x " << 2 * ROOM_HALF_WIDTH << " m\n"
              << "Encoders: " << data.wheel_radius
              << " m wheels; odometry and the filter take them "
              << "to be " << std::setprecision(4) << wheel_radius << " m\n"
              << std::setprecision(2) << "IMU: " << 1.0 / imu.backend_ptr()->sampling_period()
              << " readings/s, "
              << (options.calibrate ? "odometry measures the gyro's bias first"
                                    : "no calibration for odometry")
              << "; the filter learns the bias\n"
              << "Into the filter: infrared " << (options.use_ir ? "yes" : "no") << ", camera "
              << (options.use_camera ? "yes" : "no") << ", sonars, ahead at " << front_range.x
              << " m and behind at " << rear_range.x << " m, " << (options.use_sonar ? "yes" : "no")
              << "\n\n"
              << "    t [s]   x [m]   y [m]  ekf x [m]  ekf y [m]  ekf off [m]  odo off [m]"
                 "  ekf head off [deg]  bias [rad/s]  state\n";

    const real_t ir_period = ir.backend_ptr()->sampling_period();
    const real_t sonar_period = sonar.backend_ptr()->sampling_period();
    const real_t imu_period = imu.backend_ptr()->sampling_period();
    const real_t camera_period = 1.0 / camera.config().update_rate;
    real_t next_imu = 0.0;
    real_t next_ir = 0.0;
    real_t next_sonar = 0.0;
    real_t next_camera = 0.0;
    real_t next_print = 0.0;
    real_t next_frame = 0.0;
    Command command;

    real_t last_angle = std::atan2(START[1], START[0]);
    real_t travelled_angle = 0.0;
    uint_t n_hits = 0;
    bool was_touching = false;
    bool completed = false;
    bool stopped = false;
    bool gave_up = false;

    ErrorStats ekf_error;
    ErrorStats odometry_error;
    real_t max_ekf_heading_error = 0.0;
    Usage ir_usage;
    Usage camera_usage;
    Usage sonar_usage;

    const auto wall_start = std::chrono::steady_clock::now();
    while (world.time() < TIME_LIMIT)
    {
        // The IMU and the encoders, read together, predict
        if (world.time() + 1e-9 >= next_imu)
        {
            next_imu += imu_period;
            imu.read_values();
            const real_t gyro_z = imu.angular_velocity()[2];
            heading.update(gyro_z, imu_period);
            odometry.update(left->encoder_angle(), right->encoder_angle(), heading.heading());

            const real_t l = left->encoder_angle();
            const real_t r = right->encoder_angle();
            ekf.predict(0.5 * wheel_radius * ((l - last_left) + (r - last_right)), gyro_z,
                        imu_period);
            last_left = l;
            last_right = r;
        }
        if (world.time() + 1e-9 >= next_sonar)
        {
            next_sonar += sonar_period;
            sonar.read_values();
            rear_sonar.read_values();
            if (options.use_sonar)
            {
                for (const auto &[mount, reading] : {std::pair{front_range, sonar.distance()},
                                                     std::pair{rear_range, rear_sonar.distance()}})
                {
                    const auto result = ekf.update_range(mount, reading);
                    sonar_usage.count(result, result.accepted || result.mahalanobis > 0.0);
                }
            }
        }
        if (world.time() + 1e-9 >= next_camera)
        {
            next_camera += camera_period;
            if (options.use_camera)
            {
                const auto points = camera.line_points(robot.camera_view(floor));
                camera_usage.count(ekf.update_line(points, CAMERA_SIGMA), !points.empty());
            }
        }
        if (world.time() + 1e-9 >= next_ir)
        {
            next_ir += ir_period;
            ir.read_values();
            if (options.use_ir)
            {
                if (const auto line = ir.line_position())
                {
                    const auto p = world.from_axle(chrono::ChVector3d((*line)[0], (*line)[1], 0.0));
                    ir_usage.count(ekf.update_line({p}, IR_SIGMA), true);
                }
            }
            // Stand still while odometry measures the gyro's bias, then drive as in
            // wheel_odometry
            command = heading.is_calibrating()
                          ? Command{}
                          : driver.command(ir, sonar.distance(), odometry.distance(),
                                           heading.heading(), ir_period);
            if (driver.has_given_up())
            {
                gave_up = true;
                std::cout << "\nThe robot lost the line for " << LOST_TIME << " s and stops"
                          << std::endl;
                break;
            }
        }

        world.step(command);

        const Point p = world.position();
        const real_t angle = std::atan2(p[1], p[0]);
        travelled_angle += std::remainder(angle - last_angle, 2.0 * PI);
        last_angle = angle;
        if (world.is_touching_an_obstacle() && !was_touching)
        {
            ++n_hits;
            std::cout << "  the robot touches a box or a wall at t = " << world.time() << " s"
                      << std::endl;
        }
        was_touching = world.is_touching_an_obstacle();

        const auto e = ekf.pose();
        const auto &o = odometry.pose();
        ekf_error.add(std::hypot(e.x - p[0], e.y - p[1]));
        odometry_error.add(std::hypot(o.x - p[0], o.y - p[1]));
        const real_t heading_error = std::remainder(e.yaw - world.heading(), 2.0 * PI);
        max_ekf_heading_error = std::max(max_ekf_heading_error, std::abs(heading_error));
        if (display)
        {
            display->add_to_trails(p, {o.x, o.y}, {e.x, e.y});
        }

        if (world.time() + 1e-9 >= next_print)
        {
            next_print += PRINT_PERIOD;
            std::cout << std::setw(9) << std::setprecision(2) << world.time()
                      << std::setprecision(3) << std::setw(8) << p[0] << std::setw(8) << p[1]
                      << std::setw(11) << e.x << std::setw(11) << e.y << std::setw(13)
                      << std::hypot(e.x - p[0], e.y - p[1]) << std::setw(13)
                      << std::hypot(o.x - p[0], o.y - p[1]) << std::setw(20) << std::setprecision(2)
                      << heading_error * 180.0 / PI << std::setw(14) << std::setprecision(4)
                      << ekf.gyro_bias() << "  " << std::left
                      << (heading.is_calibrating() ? "calibrate" : to_string(driver.state()))
                      << std::right << std::endl;
        }

        if (display && world.time() + 1e-9 >= next_frame)
        {
            next_frame += 1.0 / RENDER_RATE;
            if (!display->show(sonar.distance(), rear_sonar.distance(), odometry, ekf, driver,
                               std::clamp(travelled_angle / (2.0 * PI), 0.0, 1.0)))
            {
                stopped = true;
                break;
            }
            const real_t ahead = world.time() - std::chrono::duration<real_t>(
                                                    std::chrono::steady_clock::now() - wall_start)
                                                    .count();
            if (ahead > 0.0)
            {
                std::this_thread::sleep_for(std::chrono::duration<real_t>(ahead));
            }
        }

        if (travelled_angle >= 2.0 * PI)
        {
            completed = true;
            break;
        }
    }

    const real_t wall_time =
        std::chrono::duration<real_t>(std::chrono::steady_clock::now() - wall_start).count();
    const Point p = world.position();
    const auto e = ekf.pose();
    const auto &o = odometry.pose();
    std::cout << std::setprecision(2) << "\n"
              << (completed ? "The robot went once around the track"
                  : stopped ? "Stopped"
                  : gave_up ? "The robot lost the line"
                            : "The robot did not get around the track")
              << " in " << world.time() << " s of simulation, " << wall_time << " s of wall time\n"
              << "Detours around boxes: " << driver.n_detours() << " of " << OBSTACLES.size()
              << ", times it touched a box or a wall: " << n_hits << "\n";

    const auto imu_backend =
        std::dynamic_pointer_cast<bitsim::sensors::backends::CHRONO_IMUBackend>(imu.imu_backend());
    if (imu_backend)
    {
        std::cout << std::setprecision(4)
                  << "Gyro bias about z: " << imu_backend->errors().gyro_bias[2]
                  << " rad/s; odometry measured " << heading.bias() << ", the filter "
                  << ekf.gyro_bias() << " rad/s\n";
    }
    std::cout << std::setprecision(3)
              << "Off the true position, at the end / at most / on average:\n"
              << "  odometry:   " << std::hypot(o.x - p[0], o.y - p[1]) << " / "
              << odometry_error.max << " / " << odometry_error.mean() << " m\n"
              << "  the filter: " << std::hypot(e.x - p[0], e.y - p[1]) << " / " << ekf_error.max
              << " / " << ekf_error.mean() << " m, heading at most " << std::setprecision(2)
              << max_ekf_heading_error * 180.0 / PI << " deg off\n"
              << "Readings into the filter (used / rejected by the gate): infrared "
              << ir_usage.used << " / " << ir_usage.rejected << ", camera " << camera_usage.used
              << " / " << camera_usage.rejected << ", sonars " << sonar_usage.used << " / "
              << sonar_usage.rejected << std::endl;

    return completed && n_hits == 0 ? 0 : 1;
}

} // namespace ekf_sensor_fusion

int main(int argc, char **argv)
{
    try
    {
        return ekf_sensor_fusion::run(ekf_sensor_fusion::parse(argc, argv));
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}

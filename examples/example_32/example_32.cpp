// Example 32: Following a line with infrared sensors and avoiding obstacles
// with an ultrasonic sensor
//
// The differential drive robot of example 14 drives around a track: a dark
// line on light paper. Three infrared elements under the front of its chassis
// tell it where the line is, and an ultrasonic sensor on its front tells it
// how far the nearest obstacle ahead is. Boxes stand on the track. The robot
// follows the line, and when a box is close it drives around it and comes back
// to the line on the other side.
//
// The robot also carries a camera that looks ahead. It does nothing but show,
// in the corner of the window, what is in front of the robot.
//
// Usage: example_32 [--render] [--record DIR]
//
// --render shows the simulation in a window, in real time. --record DIR also
// writes the window to DIR as frame_00000.png, frame_00001.png, ..., ten a
// second of simulation; ffmpeg turns them into a video.

#include "bitrl/bitrl_config.h"
#include "bitrl/bitrl_consts.h"
#include "bitrl/bitrl_types.h"
#include "bitrl/rigid_bodies/chrono_robots/diff_drive_robot.h"
#include "bitrl/sensors/backends/chrono_ultrasound_backend.h"
#include "bitrl/sensors/downward_camera.h"
#include "bitrl/sensors/infrared_sensor.h"
#include "bitrl/sensors/ultrasonic_sensor.h"
#include "bitrl/utils/render/irrlicht_utils.h"

#include <chrono/assets/ChVisualShapeTriangleMesh.h>
#include <chrono/geometry/ChTriangleMeshConnected.h>
#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChContactContainer.h>
#include <chrono/physics/ChContactMaterialNSC.h>
#include <chrono/physics/ChSystemNSC.h>
#include <chrono_irrlicht/ChIrrCamera.h>
#include <chrono_irrlicht/ChVisualSystemIrrlicht.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

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

namespace example_32
{

using bitrl::real_t;
using bitrl::uint_t;
using bitrl::rb::bitrl_chrono::CHRONO_DiffDriveRobot;
using bitrl::sensors::FloorImage;
using bitrl::sensors::InfraredSensor;
using bitrl::sensors::UltrasonicSensor;
using Point = std::array<real_t, 2>;

const std::string ROBOT_FILE = bitrl::consts::ROBOTS_DIR + "/bitrl_diff_drive_robot.json";
const std::string WINDOW_TITLE = "Example 32: Line following and obstacle avoidance";

const real_t PI = bitrl::consts::maths::PI;
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

// The camera that looks ahead: where it sits on the chassis, and how it looks
const chrono::ChVector3d CAMERA_POSITION{0.26, 0.0, 0.07}; // from the centre of the chassis
const real_t CAMERA_TILT = 8.0 * PI / 180;                 // [rad] down from the horizon
const real_t CAMERA_FOV = 60.0 * PI / 180;                 // [rad] vertical
const int CAMERA_WIDTH = 384;                              // [pixels] in the window
const int CAMERA_HEIGHT = 216;

// ---------------------------------------------------------------------------
// The track
// ---------------------------------------------------------------------------

// The distance from a point to the centre of the line: the distance to the
// boundary of a rounded rectangle
real_t distance_to_track(const Point &p)
{
    const real_t qx = std::abs(p[0]) - (TRACK_HALF_LENGTH - TRACK_CORNER_RADIUS);
    const real_t qy = std::abs(p[1]) - (TRACK_HALF_WIDTH - TRACK_CORNER_RADIUS);
    const real_t outside = std::hypot(std::max(qx, 0.0), std::max(qy, 0.0));
    const real_t inside = std::min(std::max(qx, qy), 0.0);
    return std::abs(outside + inside - TRACK_CORNER_RADIUS);
}

// The paper, with the line on it. It is both what the infrared elements look
// at and the texture of the floor in the window
FloorImage draw_floor()
{
    const int cols = static_cast<int>(std::round(FLOOR_WIDTH / METRES_PER_PIXEL));
    const int rows = static_cast<int>(std::round(FLOOR_HEIGHT / METRES_PER_PIXEL));
    cv::Mat image(rows, cols, CV_8UC1, cv::Scalar(PAPER));
    FloorImage floor(image, FLOOR_WIDTH);
    for (int r = 0; r < rows; ++r)
    {
        for (int c = 0; c < cols; ++c)
        {
            if (distance_to_track(floor.to_world(cv::Point2d(c, r))) < 0.5 * LINE_WIDTH)
            {
                image.at<uchar>(r, c) = INK;
            }
        }
    }
    return floor;
}

// ---------------------------------------------------------------------------
// Following the line
// ---------------------------------------------------------------------------

struct Command
{
    real_t v{0.0};
    real_t omega{0.0};
};

// The three elements, left to right, say where the line is: under the left
// one, the line is to the left, and so on. The robot turns towards the line,
// harder the further out it is. With no element on the line it turns on the
// spot towards where it saw the line last
class LineFollower
{
  public:
    Command command(const InfraredSensor &ir, real_t dt)
    {
        // Where the line is: +1 under the left element, 0 under the middle
        // one, -1 under the right one
        real_t sum = 0.0;
        uint_t n_dark = 0;
        for (uint_t i = 0; i < ir.n_elements(); ++i)
        {
            if (ir.is_dark(i))
            {
                sum += 1.0 - 2.0 * i / (ir.n_elements() - 1.0);
                ++n_dark;
            }
        }

        if (n_dark == 0)
        {
            lost_for_ += dt;
            return {0.0, last_side_ >= 0.0 ? SEARCH_TURN_RATE : -SEARCH_TURN_RATE};
        }
        lost_for_ = 0.0;

        const real_t side = sum / n_dark;
        if (side != 0.0)
        {
            last_side_ = side;
        }
        return {CRUISE_SPEED * (1.0 - 0.5 * std::abs(side)), LINE_TURN_RATE * side};
    }

    bool is_lost() const { return lost_for_ > 0.0; }
    bool has_given_up() const { return lost_for_ > LOST_TIME; }

  private:
    real_t last_side_{0.0};
    real_t lost_for_{0.0};
};

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

    Command command(const InfraredSensor &ir, real_t sonar, const Point &position, real_t heading,
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
                return command(ir, sonar, position, heading, dt);
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
                cross_from_ = position;
                state_ = State::CROSS;
                return command(ir, sonar, position, heading, dt);
            }
            return {DETOUR_SPEED, DETOUR_SPEED / DETOUR_RADIUS};

        case State::CROSS:
            if (std::hypot(position[0] - cross_from_[0], position[1] - cross_from_[1]) < ir_ahead_)
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
    LineFollower follower_;
    State state_{State::FOLLOW};
    real_t turn_to_{0.0};
    Point cross_from_{0.0, 0.0};
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

// "#.#": which elements see the line, left to right
std::string line_pattern(const InfraredSensor &ir)
{
    std::string pattern;
    for (uint_t i = 0; i < ir.n_elements(); ++i)
    {
        pattern += ir.is_dark(i) ? '#' : '.';
    }
    return pattern;
}

// ---------------------------------------------------------------------------
// The simulated world
// ---------------------------------------------------------------------------

// Counts the contacts between the robot and the boxes
class ObstacleContacts : public chrono::ChContactContainer::ReportContactCallback
{
  public:
    explicit ObstacleContacts(std::vector<std::shared_ptr<chrono::ChBody>> obstacles)
        : obstacles_(std::move(obstacles))
    {
    }

    bool OnReportContact(const chrono::ChVector3d &, const chrono::ChVector3d &,
                         const chrono::ChMatrix33<> &, double distance, double,
                         const chrono::ChVector3d &, const chrono::ChVector3d &,
                         chrono::ChContactable *a, chrono::ChContactable *b, int) override
    {
        if (distance < 0.0 && (is_obstacle_(a) || is_obstacle_(b)))
        {
            ++n_contacts;
        }
        return true;
    }

    uint_t n_contacts{0};

  private:
    bool is_obstacle_(const chrono::ChContactable *c) const
    {
        return std::any_of(obstacles_.begin(), obstacles_.end(),
                           [c](const auto &body)
                           { return static_cast<const chrono::ChContactable *>(body.get()) == c; });
    }

    std::vector<std::shared_ptr<chrono::ChBody>> obstacles_;
};

class World
{
  public:
    explicit World(const FloorImage &floor)
    {
        system_.SetGravitationalAcceleration(
            chrono::ChVector3d(0.0, 0.0, -bitrl::consts::maths::G));
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
        contacts_ = chrono_types::make_shared<ObstacleContacts>(obstacles_);

        robot_.load(std::filesystem::path(ROBOT_FILE));
        if (!robot_.get_line_sensor() || !robot_.get_sonar())
        {
            throw std::runtime_error("The robot file " + ROBOT_FILE +
                                     " needs a line-sensor and a front-sonar");
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

        contacts_->n_contacts = 0;
        system_.GetContactContainer()->ReportAllContacts(contacts_);
        touching_ = contacts_->n_contacts > 0;
    }

    // Put the paper on the ground, so that the window shows the line: a
    // rectangle just above the ground with the picture on it. The top left
    // corner of the picture is the corner of the floor with the smallest x and
    // the largest y
    void show_floor(const std::string &texture)
    {
        auto mesh = chrono_types::make_shared<chrono::ChTriangleMeshConnected>();
        const real_t x = 0.5 * FLOOR_WIDTH;
        const real_t y = 0.5 * FLOOR_HEIGHT;
        const real_t z = 0.051; // above the centre of the ground, which is 0.1 thick
        mesh->GetCoordsVertices() = {{-x, y, z}, {x, y, z}, {x, -y, z}, {-x, -y, z}};
        // Irrlicht's v runs up the picture
        mesh->GetCoordsUV() = {{0.0, 1.0}, {1.0, 1.0}, {1.0, 0.0}, {0.0, 0.0}};
        mesh->GetCoordsNormals() = {{0.0, 0.0, 1.0}};
        mesh->GetIndicesVertexes() = {{3, 2, 1}, {3, 1, 0}};
        mesh->GetIndicesUV() = {{3, 2, 1}, {3, 1, 0}};
        mesh->GetIndicesNormals() = {{0, 0, 0}, {0, 0, 0}};

        auto paper = chrono_types::make_shared<chrono::ChVisualShapeTriangleMesh>();
        paper->SetMesh(mesh);
        paper->SetTexture(texture);
        ground_->AddVisualShape(paper);
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

  private:
    chrono::ChSystemNSC system_;
    std::shared_ptr<chrono::ChBodyEasyBox> ground_;
    std::vector<std::shared_ptr<chrono::ChBody>> obstacles_;
    std::shared_ptr<ObstacleContacts> contacts_;
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
        : world_(world), record_(record)
    {
        // The paper on the ground, as a texture. The visual system takes the
        // shapes of the bodies when it starts, so it goes on first.
        // Irrlicht reads colour pictures more reliably than grey ones
        texture_ = (std::filesystem::temp_directory_path() / "bitrl_example_32_floor.png").string();
        cv::Mat colour;
        cv::cvtColor(floor.image(), colour, cv::COLOR_GRAY2BGR);
        if (!cv::imwrite(texture_, colour))
        {
            throw std::runtime_error("Cannot write the floor texture " + texture_);
        }
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

        // The robot's camera: input from the mouse moves the main camera only.
        // A new camera makes itself the active one, so remember the main one
        auto *smgr = vis_.GetSceneManager();
        main_camera_ = vis_.GetActiveCamera();
        robot_camera_ =
            new chrono::irrlicht::RTSCamera(vis_.GetDevice(), smgr->getRootSceneNode(), smgr, -1);
        robot_camera_->setInputReceiverEnabled(false);
        robot_camera_->setZUp();
        robot_camera_->setFOV(static_cast<irr::f32>(CAMERA_FOV));
        robot_camera_->setAspectRatio(static_cast<irr::f32>(CAMERA_WIDTH) / CAMERA_HEIGHT);
        robot_camera_->setNearValue(0.01f);
        robot_camera_->drop();
        smgr->setActiveCamera(main_camera_);

        if (!record_.empty())
        {
            std::filesystem::create_directories(record_);
        }
    }

    // Returns false once the window is closed
    bool show(const InfraredSensor &ir, real_t sonar, const Driver &driver, const Command &command,
              real_t lap)
    {
        if (!vis_.Run())
        {
            return false;
        }

        vis_.BeginScene();
        vis_.Render();
        draw_sonar_(sonar);
        draw_robot_view_();
        draw_text_(ir, sonar, driver, command, lap);
        vis_.EndScene();

        if (!record_.empty() && world_.time() + 1e-9 >= next_record_)
        {
            next_record_ += 1.0 / RECORD_RATE;
            std::ostringstream name;
            name << "frame_" << std::setw(5) << std::setfill('0') << n_recorded_++ << ".png";
            vis_.WriteImageToFile((std::filesystem::path(record_) / name.str()).string());
        }
        return true;
    }

  private:
    // The sonar's beam, as far as the echo came from
    void draw_sonar_(real_t sonar)
    {
        const auto chassis = world_.robot().get_chassis();
        const auto &frame = chassis->GetFrameRefToAbs();
        const auto sonar_backend =
            std::dynamic_pointer_cast<bitrl::sensors::backends::CHRONO_UltrasonicBackend>(
                world_.robot().get_sonar()->range_backend());
        const chrono::ChVector3d start = frame.TransformPointLocalToParent(
            sonar_backend ? sonar_backend->position() : chrono::ChVector3d(0.27, 0, 0.05));
        const chrono::ChVector3d dir = frame.TransformDirectionLocalToParent({1.0, 0.0, 0.0});
        const auto colour = sonar < STOP_DISTANCE ? chrono::ChColor(1.0f, 0.1f, 0.1f)
                                                  : chrono::ChColor(0.2f, 0.8f, 0.2f);
        bitrl::utils::render::irr_draw_line(vis_, start, start + dir * sonar, colour);
    }

    // Render the scene again, into the corner, from the robot's camera
    void draw_robot_view_()
    {
        const auto &frame = world_.robot().get_chassis()->GetFrameRefToAbs();
        const chrono::ChVector3d eye = frame.TransformPointLocalToParent(CAMERA_POSITION);
        const chrono::ChVector3d look = frame.TransformDirectionLocalToParent(
            {std::cos(CAMERA_TILT), 0.0, -std::sin(CAMERA_TILT)});
        const chrono::ChVector3d target = eye + look;
        robot_camera_->setPosition(irr::core::vector3df(eye.x(), eye.y(), eye.z()));
        robot_camera_->setTarget(irr::core::vector3df(target.x(), target.y(), target.z()));

        auto *driver = vis_.GetVideoDriver();
        auto *smgr = vis_.GetSceneManager();
        const irr::core::rect<irr::s32> whole = driver->getViewPort();
        const irr::core::rect<irr::s32> inset(10, 10, 10 + CAMERA_WIDTH, 10 + CAMERA_HEIGHT);

        driver->setViewPort(inset);
        driver->clearZBuffer();
        smgr->setActiveCamera(robot_camera_);
        smgr->drawAll();
        smgr->setActiveCamera(main_camera_);
        driver->setViewPort(whole);

        driver->draw2DRectangleOutline(inset, irr::video::SColor(255, 230, 230, 230));
        font_()->draw(L"robot camera",
                      irr::core::rect<irr::s32>(14, 14 + CAMERA_HEIGHT, 300, 30 + CAMERA_HEIGHT),
                      irr::video::SColor(255, 255, 255, 255));
    }

    void draw_text_(const InfraredSensor &ir, real_t sonar, const Driver &driver,
                    const Command &command, real_t lap)
    {
        std::ostringstream text;
        text << std::fixed << std::setprecision(1) << "t " << world_.time() << " s    lap "
             << 100.0 * lap << " %    infrared " << line_pattern(ir) << "    sonar "
             << std::setprecision(2) << sonar << " m    " << to_string(driver.state()) << "    v "
             << command.v << " m/s    w " << command.omega << " rad/s    detours "
             << driver.n_detours();
        const std::string s = text.str();
        auto *driver_ = vis_.GetVideoDriver();
        const auto size = driver_->getScreenSize();
        const irr::core::rect<irr::s32> bar(0, size.Height - 26, size.Width, size.Height);
        driver_->draw2DRectangle(irr::video::SColor(200, 250, 250, 250), bar);
        font_()->draw(std::wstring(s.begin(), s.end()).c_str(),
                      irr::core::rect<irr::s32>(10, size.Height - 20, size.Width, size.Height),
                      irr::video::SColor(255, 20, 20, 20));
    }

    irr::gui::IGUIFont *font_() { return vis_.GetGUIEnvironment()->getSkin()->getFont(); }

    World &world_;
    std::string record_;
    std::string texture_;
    chrono::irrlicht::ChVisualSystemIrrlicht vis_;
    chrono::irrlicht::RTSCamera *robot_camera_{nullptr};
    irr::scene::ICameraSceneNode *main_camera_{nullptr};
    real_t next_record_{0.0};
    uint_t n_recorded_{0};
};

// ---------------------------------------------------------------------------
// Running it
// ---------------------------------------------------------------------------

struct Options
{
    bool render{false};
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
        else if (arg == "--record" && i + 1 < argc)
        {
            options.record = argv[++i];
            options.render = true;
        }
        else
        {
            std::cout << "Usage: example_32 [--render] [--record DIR]\n"
                      << "  --render        show the simulation in a window, in real time\n"
                      << "  --record DIR    also write the window to DIR, " << RECORD_RATE
                      << " frames a second" << std::endl;
            std::exit(arg == "--help" || arg == "-h" ? 0 : 1);
        }
    }
    return options;
}

int run(const Options &options)
{
    const FloorImage floor = draw_floor();
    World world(floor);
    auto &robot = world.robot();
    auto &ir = *robot.get_line_sensor();
    auto &sonar = *robot.get_sonar();

    std::optional<Display> display;
    if (options.render)
    {
        display.emplace(world, floor, options.record);
    }

    Driver driver(world.ir_ahead());
    std::cout << std::fixed << std::setprecision(2) << "Track " << 2 * TRACK_HALF_LENGTH << " x "
              << 2 * TRACK_HALF_WIDTH << " m, line " << LINE_WIDTH << " m wide, "
              << OBSTACLES.size() << " boxes on it\n"
              << "Infrared: " << ir.n_elements() << " elements, " << world.ir_ahead()
              << " m ahead of the axle, " << 1.0 / ir.backend_ptr()->sampling_period()
              << " readings/s\n"
              << "Sonar: " << sonar.range_backend()->min_distance() << " to "
              << sonar.range_backend()->max_distance() << " m, "
              << 1.0 / sonar.backend_ptr()->sampling_period() << " readings/s\n\n"
              << "    t [s]   x [m]   y [m]  head [deg]  infrared  sonar [m]  state      "
                 "v [m/s]  w [rad/s]\n";

    const real_t ir_period = ir.backend_ptr()->sampling_period();
    const real_t sonar_period = sonar.backend_ptr()->sampling_period();
    real_t next_ir = 0.0;
    real_t next_sonar = 0.0;
    real_t next_print = 0.0;
    real_t next_frame = 0.0;
    Command command;

    // Progress around the track, how long the line was under the elements
    // while following it, and whether the robot hit a box
    real_t last_angle = std::atan2(START[1], START[0]);
    real_t travelled_angle = 0.0;
    real_t follow_time = 0.0;
    real_t lost_time = 0.0;
    real_t longest_lost = 0.0;
    real_t lost_since = 0.0;
    uint_t n_hits = 0;
    bool was_touching = false;
    bool completed = false;
    bool stopped = false;
    bool gave_up = false;

    const auto wall_start = std::chrono::steady_clock::now();
    while (world.time() < TIME_LIMIT)
    {
        // Each sensor reads at its own rate. The robot decides what to do
        // whenever the infrared elements read
        if (world.time() + 1e-9 >= next_sonar)
        {
            next_sonar += sonar_period;
            sonar.read_values();
        }
        if (world.time() + 1e-9 >= next_ir)
        {
            next_ir += ir_period;
            ir.read_values();
            command =
                driver.command(ir, sonar.distance(), world.position(), world.heading(), ir_period);
            if (driver.has_given_up())
            {
                gave_up = true;
                std::cout << "\nThe robot lost the line for " << LOST_TIME << " s and stops"
                          << std::endl;
                break;
            }
        }

        world.step(command);

        // Statistics
        const Point p = world.position();
        const real_t angle = std::atan2(p[1], p[0]);
        travelled_angle += std::remainder(angle - last_angle, 2.0 * PI);
        last_angle = angle;
        if (driver.state() == Driver::State::FOLLOW)
        {
            follow_time += STEP_SIZE;
            if (driver.is_lost())
            {
                lost_time += STEP_SIZE;
                lost_since += STEP_SIZE;
                longest_lost = std::max(longest_lost, lost_since);
            }
            else
            {
                lost_since = 0.0;
            }
        }
        if (world.is_touching_an_obstacle() && !was_touching)
        {
            ++n_hits;
            std::cout << "  the robot touches a box at t = " << world.time() << " s" << std::endl;
        }
        was_touching = world.is_touching_an_obstacle();

        if (world.time() + 1e-9 >= next_print)
        {
            next_print += PRINT_PERIOD;
            std::cout << std::setw(9) << std::setprecision(2) << world.time()
                      << std::setprecision(3) << std::setw(8) << p[0] << std::setw(8) << p[1]
                      << std::setw(12) << std::setprecision(1) << world.heading() * 180.0 / PI
                      << std::setw(10) << line_pattern(ir) << std::setw(11) << std::setprecision(2)
                      << sonar.distance() << "  " << std::left << std::setw(10)
                      << to_string(driver.state()) << std::right << std::setw(8) << command.v
                      << std::setw(11) << command.omega << std::endl;
        }

        if (display && world.time() + 1e-9 >= next_frame)
        {
            next_frame += 1.0 / RENDER_RATE;
            if (!display->show(ir, sonar.distance(), driver, command,
                               std::clamp(travelled_angle / (2.0 * PI), 0.0, 1.0)))
            {
                stopped = true;
                break;
            }
            // Keep to real time, so that the window shows the robot at its speed
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
    std::cout << std::setprecision(2) << "\n"
              << (completed ? "The robot went once around the track"
                  : stopped ? "Stopped"
                  : gave_up ? "The robot lost the line"
                            : "The robot did not get around the track")
              << " in " << world.time() << " s of simulation, " << wall_time << " s of wall time\n"
              << "Detours around boxes: " << driver.n_detours() << " of " << OBSTACLES.size()
              << ", times it touched a box: " << n_hits << "\n"
              << "While following the line, the elements saw it "
              << 100.0 * (1.0 - lost_time / std::max(follow_time, STEP_SIZE))
              << " % of the time; the longest time without it was " << longest_lost << " s"
              << std::endl;

    return completed && n_hits == 0 ? 0 : 1;
}

} // namespace example_32

int main(int argc, char **argv)
{
    try
    {
        return example_32::run(example_32::parse(argc, argv));
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}

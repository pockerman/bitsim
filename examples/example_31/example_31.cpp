// Example 31: A line following robot that sees through a Raspberry Pi camera
//
// A Raspberry Pi 5 streams pictures of a line drawn on paper over HTTP. Each
// picture is the floor of the simulated world. The differential drive robot of
// example 14 carries a camera that looks down at the floor just ahead of it;
// it sees a patch of the latest picture, finds the line in it and steers to
// follow it.
//
// The pictures come from one of two endpoints of the Pi's server:
//
//   /odysseus/camera/square-path  (the default) generated pictures, no camera.
//       Every few seconds the picture shows the next side of a square. The
//       robot follows the side it sees, waits at its end, and turns onto the
//       next side when it appears, so it drives around the square side by side
//   /odysseus/camera              (--pi-camera) the Pi camera, looking down at
//       a square drawn on paper. The robot follows the line around it
//
// Usage: example_31 [--pi-camera] [--render] [--record FILE] [--floor-width W] [server URL]
//
// --floor-width is how wide, in metres, the floor that a picture covers is;
// 3 by default. Without a Pi, run camera_server.py --synthetic on this
// computer and pass http://localhost:8000.

#include "bitrl/bitrl_config.h"
#include "bitrl/bitrl_consts.h"
#include "bitrl/bitrl_types.h"
#include "bitrl/rigid_bodies/chrono_robots/diff_drive_robot.h"
#include "bitrl/sensors/backends/mjpeg_http_camera_backend.h"
#include "bitrl/sensors/downward_camera.h"

#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChContactMaterialNSC.h>
#include <chrono/physics/ChSystemNSC.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace example_31
{

using bitrl::real_t;
using bitrl::uint_t;
using bitrl::rb::bitrl_chrono::CHRONO_DiffDriveRobot;
using bitrl::sensors::DownwardCamera;
using bitrl::sensors::FloorImage;
using bitrl::sensors::backends::CameraFrame;
using bitrl::sensors::backends::MJPEG_HTTPCameraBackend;
using Point = std::array<real_t, 2>;

const std::string DEFAULT_SERVER = "http://alex-rpi-5.local:8000";
const std::string SQUARE_PATH_ENDPOINT = "/odysseus/camera/square-path";
const std::string CAMERA_ENDPOINT = "/odysseus/camera";
const std::string ROBOT_FILE = bitrl::consts::ROBOTS_DIR + "/bitrl_diff_drive_robot.json";
const std::string WINDOW_TITLE = "Example 31: A line following robot with a Raspberry Pi camera";

const real_t PI = bitrl::consts::maths::PI;
const real_t STEP_SIZE = 0.01;          // [s]
const real_t DEFAULT_FLOOR_WIDTH = 3.0; // [m]
const real_t TIME_LIMIT = 120.0;        // [s]
const real_t PRINT_PERIOD = 1.0;        // [s]

// The line follower
const real_t CRUISE_SPEED = 0.25;      // [m/s]
const real_t MIN_SPEED_RATIO = 0.3;    // of the cruise speed, in sharp turns
const real_t MAX_TURN_RATE = 1.5;      // [rad/s]
const real_t SEARCH_TURN_RATE = 0.6;   // [rad/s]
const real_t LOST_TIME = 5.0;          // [s] with no line before giving up, Pi camera
const real_t WAIT_TIME = 15.0;         // [s] to wait for the next side, square path
const real_t MIN_LINE_COVERAGE = 0.01; // of the camera image

// On the square path each side, with its corner, has to be done before the
// next side appears, so the robot drives faster there
const real_t SQUARE_PATH_SPEED = 0.35; // [m/s]
const real_t CORNER_TURN_RATE = 1.5;   // [rad/s]
const real_t STOP_GAIN = 2.0;          // [1/s] how quickly to slow down for the corner
const real_t DARK_RATIO = 0.6;         // darker than this times the paper is line

// ---------------------------------------------------------------------------
// Seeing the line
// ---------------------------------------------------------------------------

cv::Mat to_gray(const cv::Mat &image)
{
    if (image.channels() == 1)
    {
        return image;
    }
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    return gray;
}

// The pixels much darker than the paper. The paper is the brightness that
// 90 % of the pixels are darker than, so uneven lighting does not matter much
cv::Mat dark_pixels(const cv::Mat &gray)
{
    std::vector<uchar> values(gray.begin<uchar>(), gray.end<uchar>());
    auto paper = values.begin() + static_cast<long>(0.9 * (values.size() - 1));
    std::nth_element(values.begin(), paper, values.end());
    return gray < DARK_RATIO * (*paper);
}

// What the robot's camera makes of its image
struct LineReading
{
    bool found{false};
    real_t forward{0.0};  // where the middle of the line it sees is: ahead of the axle [m]
    real_t left{0.0};     // and to its left [m]
    real_t coverage{0.0}; // the part of the image that is line
    cv::Point2d pixel;    // the middle of the line in the image
    bool ends{false};     // whether the line ends inside the image, before its top
    real_t end{0.0};      // how far ahead of the axle the far end of the line is [m]
};

LineReading read_line(const cv::Mat &view, const DownwardCamera &camera)
{
    const cv::Mat dark = dark_pixels(to_gray(view));
    const cv::Moments m = cv::moments(dark, true);

    LineReading reading;
    reading.coverage = m.m00 / static_cast<real_t>(dark.total());
    if (reading.coverage < MIN_LINE_COVERAGE)
    {
        return reading;
    }
    reading.found = true;
    reading.pixel = cv::Point2d(m.m10 / m.m00, m.m01 / m.m00);
    const auto where = camera.to_robot(reading.pixel.x, reading.pixel.y);
    reading.forward = where[0];
    reading.left = where[1];

    // The far end of the line is its top row in the image
    int top = 0;
    while (top < dark.rows && cv::countNonZero(dark.row(top)) == 0)
    {
        ++top;
    }
    reading.ends = top > 2;
    reading.end = camera.to_robot(reading.pixel.x, top)[0];
    return reading;
}

// ---------------------------------------------------------------------------
// The line on the paper
// ---------------------------------------------------------------------------

// A straight stretch of the centre of the line, in world coordinates
struct Segment
{
    Point a;
    Point b;

    Point middle() const { return {0.5 * (a[0] + b[0]), 0.5 * (a[1] + b[1])}; }
    real_t length() const { return std::hypot(b[0] - a[0], b[1] - a[1]); }

    real_t distance(const Point &p) const
    {
        const real_t dx = b[0] - a[0];
        const real_t dy = b[1] - a[1];
        const real_t s =
            std::clamp(((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / (dx * dx + dy * dy), 0.0, 1.0);
        return std::hypot(p[0] - a[0] - s * dx, p[1] - a[1] - s * dy);
    }
};

// The line the robot should follow, as far as the simulator knows it: the
// whole square with the Pi camera, the sides seen so far on the square path.
// It is used to start the robot and to measure how well it follows the line;
// the robot itself sees only through its camera
struct Path
{
    std::vector<Segment> segments;
    Point centre{0.0, 0.0};
    real_t line_width{0.0};

    real_t distance(const Point &p) const
    {
        real_t best = std::numeric_limits<real_t>::max();
        for (const auto &segment : segments)
        {
            best = std::min(best, segment.distance(p));
        }
        return best;
    }

    // Add a side, unless it is one already known
    void add(const Segment &segment)
    {
        for (const auto &known : segments)
        {
            const Point m = segment.middle();
            const Point k = known.middle();
            if (std::hypot(m[0] - k[0], m[1] - k[1]) < 0.5 * line_width + 0.05)
            {
                return;
            }
        }
        segments.push_back(segment);
    }
};

// Find the square: the largest dark outline with a hole in it. The centre
// line is halfway between its outer and inner edges. Its sides go
// counterclockwise
std::optional<Path> find_square(const FloorImage &floor)
{
    cv::Mat dark = dark_pixels(to_gray(floor.image()));
    cv::morphologyEx(dark, dark, cv::MORPH_CLOSE,
                     cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(dark, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

    int outer = -1;
    int inner = -1;
    double outer_area = 0.0;
    for (int i = 0; i < static_cast<int>(contours.size()); ++i)
    {
        const bool is_outline = hierarchy[i][3] < 0 && hierarchy[i][2] >= 0;
        const double area = cv::contourArea(contours[i]);
        if (!is_outline || area <= outer_area)
        {
            continue;
        }
        // Its largest hole
        int hole = -1;
        double hole_area = 0.0;
        for (int child = hierarchy[i][2]; child >= 0; child = hierarchy[child][0])
        {
            const double a = cv::contourArea(contours[child]);
            if (a > hole_area)
            {
                hole = child;
                hole_area = a;
            }
        }
        if (hole >= 0 && hole_area > 0.3 * area)
        {
            outer = i;
            inner = hole;
            outer_area = area;
        }
    }
    if (outer < 0 || outer_area < 0.02 * static_cast<double>(dark.total()))
    {
        return std::nullopt;
    }

    const cv::RotatedRect a = cv::minAreaRect(contours[outer]);
    const cv::RotatedRect b = cv::minAreaRect(contours[inner]);
    const cv::RotatedRect centre_line(
        (a.center + b.center) * 0.5,
        cv::Size2f((a.size.width + b.size.width) * 0.5f, (a.size.height + b.size.height) * 0.5f),
        a.angle);
    const real_t aspect = centre_line.size.width / centre_line.size.height;
    if (aspect < 0.75 || aspect > 1.33)
    {
        return std::nullopt;
    }

    cv::Point2f pixels[4];
    centre_line.points(pixels);
    Path square;
    square.centre = floor.to_world(cv::Point2d(centre_line.center));
    std::array<std::pair<real_t, Point>, 4> by_angle;
    for (uint_t i = 0; i < 4; ++i)
    {
        const Point p = floor.to_world(cv::Point2d(pixels[i]));
        by_angle[i] = {std::atan2(p[1] - square.centre[1], p[0] - square.centre[0]), p};
    }
    std::sort(by_angle.begin(), by_angle.end(),
              [](const auto &l, const auto &r) { return l.first < r.first; });
    for (uint_t i = 0; i < 4; ++i)
    {
        square.segments.push_back({by_angle[i].second, by_angle[(i + 1) % 4].second});
    }
    square.line_width = 0.25 * ((a.size.width - b.size.width) + (a.size.height - b.size.height)) *
                        floor.metres_per_pixel();
    return square;
}

// Find a single straight line: the largest dark blob that is long and thin.
// Its ends are where the centre of the line ends, inside its rounded caps
std::optional<std::pair<Segment, real_t>> find_segment(const FloorImage &floor)
{
    const cv::Mat dark = dark_pixels(to_gray(floor.image()));
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(dark, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    const std::vector<cv::Point> *largest = nullptr;
    double largest_area = 0.0;
    for (const auto &contour : contours)
    {
        const double area = cv::contourArea(contour);
        if (area > largest_area)
        {
            largest = &contour;
            largest_area = area;
        }
    }
    if (!largest || largest_area < 0.002 * static_cast<double>(dark.total()))
    {
        return std::nullopt;
    }

    const cv::RotatedRect rect = cv::minAreaRect(*largest);
    const real_t length = std::max(rect.size.width, rect.size.height);
    const real_t width = std::min(rect.size.width, rect.size.height);
    if (length < 4.0 * width)
    {
        return std::nullopt;
    }
    // The direction of the long side of the rectangle
    const real_t angle =
        (rect.size.width >= rect.size.height ? rect.angle : rect.angle + 90.0) * PI / 180.0;
    const cv::Point2d along(std::cos(angle), std::sin(angle));
    const cv::Point2d centre(rect.center);
    const real_t half = 0.5 * (length - width);
    const Segment segment{floor.to_world(centre - half * along),
                          floor.to_world(centre + half * along)};
    return std::make_pair(segment, width * floor.metres_per_pixel());
}

// The direction of the side of a path nearest to a point: the side a robot
// there is following. It need not be the latest side; the next one can appear
// before the robot reaches the end of the one it is on
real_t nearest_side_heading(const Path &path, const Point &p)
{
    const Segment *nearest = &path.segments.front();
    for (const auto &segment : path.segments)
    {
        if (segment.distance(p) < nearest->distance(p))
        {
            nearest = &segment;
        }
    }
    return std::atan2(nearest->b[1] - nearest->a[1], nearest->b[0] - nearest->a[0]);
}

// Where the robot starts, and which way it faces
struct Start
{
    real_t x;
    real_t y;
    real_t yaw;
};

// With the Pi camera: the middle of the bottom side of the square, facing
// along it so that the inside of the square is on its left
Start start_on_square(const Path &square)
{
    const Segment *bottom = &square.segments.front();
    for (const auto &segment : square.segments)
    {
        if (segment.middle()[1] < bottom->middle()[1])
        {
            bottom = &segment;
        }
    }
    const Point m = bottom->middle();
    return {m[0], m[1], std::atan2(bottom->b[1] - bottom->a[1], bottom->b[0] - bottom->a[0])};
}

// On the square path: the side goes counterclockwise around the centre of the
// picture, so the centre is on the left of a robot that drives along it. Turn
// the side that way round and start at its beginning
Segment counterclockwise(const Segment &side, const Point &centre)
{
    const real_t dx = side.b[0] - side.a[0];
    const real_t dy = side.b[1] - side.a[1];
    const Point m = side.middle();
    const real_t cross = dx * (centre[1] - m[1]) - dy * (centre[0] - m[0]);
    return cross >= 0.0 ? side : Segment{side.b, side.a};
}

Start start_on_side(const Segment &side)
{
    return {side.a[0], side.a[1], std::atan2(side.b[1] - side.a[1], side.b[0] - side.a[0])};
}

// ---------------------------------------------------------------------------
// The line follower
// ---------------------------------------------------------------------------

struct Command
{
    real_t v{0.0};
    real_t omega{0.0};
};

// Pure pursuit: drive along the circle through the middle of the line the
// camera sees. In a corner the line bends away, its middle moves sideways and
// the robot turns. Slow down in sharp turns.
//
// When the camera loses the line, the robot either turns towards where it last
// saw it, to look for it, or stops and waits for the line to come back. On the
// square path the line ends at each corner until the next side appears, so
// there it waits
class LineFollower
{
  public:
    enum class WhenLost
    {
        SEARCH,
        WAIT
    };

    LineFollower(real_t half_view_width, real_t cruise_speed, WhenLost when_lost,
                 real_t give_up_after)
        : half_view_width_(half_view_width), cruise_speed_(cruise_speed), when_lost_(when_lost),
          give_up_after_(give_up_after)
    {
    }

    Command command(const LineReading &line, real_t dt)
    {
        if (!line.found)
        {
            lost_for_ += dt;
            if (when_lost_ == WhenLost::WAIT)
            {
                return {};
            }
            return {0.0, last_left_ >= 0.0 ? SEARCH_TURN_RATE : -SEARCH_TURN_RATE};
        }
        lost_for_ = 0.0;
        last_left_ = line.left;

        const real_t curvature =
            2.0 * line.left / (line.forward * line.forward + line.left * line.left);
        const real_t sideways = std::min(std::abs(line.left) / half_view_width_, 1.0);
        const real_t v = cruise_speed_ * std::max(1.0 - sideways, MIN_SPEED_RATIO);
        return {v, std::clamp(v * curvature, -MAX_TURN_RATE, MAX_TURN_RATE)};
    }

    bool has_given_up() const { return lost_for_ > give_up_after_; }
    real_t give_up_after() const { return give_up_after_; }

  private:
    real_t half_view_width_;
    real_t cruise_speed_;
    WhenLost when_lost_;
    real_t give_up_after_;
    real_t last_left_{0.0};
    real_t lost_for_{0.0};
};

// On the square path the line ends at every corner until the next side
// appears, to the left: the path goes counterclockwise. So the robot follows
// the side it sees and, once the end of the side comes into view, remembers
// where the corner is. When the end has passed under the camera it drives on
// to the corner by its own pose, turns left by a quarter turn and waits. The
// next side then starts under the robot and runs straight ahead of it
class SquarePathDriver
{
  public:
    enum class State
    {
        FOLLOW,
        TO_CORNER,
        TURN,
        WAIT
    };

    SquarePathDriver(LineFollower follower, real_t near_edge, real_t half_line_width)
        : follower_(follower), near_edge_(near_edge), half_line_width_(half_line_width)
    {
    }

    // side_heading is the direction of the side the robot is on
    Command command(const LineReading &line, const Point &position, real_t heading,
                    real_t side_heading, real_t dt)
    {
        const Point ahead{std::cos(heading), std::sin(heading)};
        switch (state_)
        {
        case State::FOLLOW:
            if (line.found)
            {
                followed_heading_ = side_heading;
            }
            if (line.found && line.ends)
            {
                // The corner is half a line width short of the end of the line
                const real_t to_corner = line.end - half_line_width_;
                corner_ =
                    Point{position[0] + to_corner * ahead[0], position[1] + to_corner * ahead[1]};
            }
            if (corner_ && (!line.found || line.end < near_edge_ + half_line_width_))
            {
                state_ = State::TO_CORNER;
                break;
            }
            return follower_.command(line, dt);

        case State::TO_CORNER:
        {
            const real_t remaining =
                ((*corner_)[0] - position[0]) * ahead[0] + ((*corner_)[1] - position[1]) * ahead[1];
            if (remaining > 0.01)
            {
                return {std::clamp(STOP_GAIN * remaining, 0.05, SQUARE_PATH_SPEED), 0.0};
            }
            // The next side leaves the corner at right angles to the side the
            // robot followed, whichever way the robot happens to face
            turn_to_ = followed_heading_ + 0.5 * PI;
            state_ = State::TURN;
            break;
        }

        case State::TURN:
            if (std::abs(std::remainder(turn_to_ - heading, 2.0 * PI)) > 0.03)
            {
                const real_t error = std::remainder(turn_to_ - heading, 2.0 * PI);
                // Ease off near the heading: the motors take a moment to stop
                return {0.0, std::clamp(1.5 * error, -CORNER_TURN_RATE, CORNER_TURN_RATE)};
            }
            state_ = State::WAIT;
            break;

        case State::WAIT:
            break;
        }

        if (state_ == State::WAIT)
        {
            if (line.found)
            {
                corner_.reset();
                state_ = State::FOLLOW;
                return follower_.command(line, dt);
            }
            waited_ += dt;
            return {};
        }
        return command(line, position, heading, side_heading, dt);
    }

    State state() const noexcept { return state_; }
    bool has_given_up() const { return follower_.has_given_up() || waited_ > WAIT_TIME; }
    real_t give_up_after() const { return WAIT_TIME; }

  private:
    LineFollower follower_;
    real_t near_edge_;
    real_t half_line_width_;
    State state_{State::FOLLOW};
    std::optional<Point> corner_;
    real_t turn_to_{0.0};
    real_t followed_heading_{0.0};
    real_t waited_{0.0};
};

// ---------------------------------------------------------------------------
// The simulated world
// ---------------------------------------------------------------------------

class World
{
  public:
    explicit World(const Start &start)
    {
        system_.SetGravitationalAcceleration(
            chrono::ChVector3d(0.0, 0.0, -bitrl::consts::maths::G));
        system_.SetCollisionSystemType(chrono::ChCollisionSystem::Type::BULLET);
        // The camera runs in real time, so the solver has to be quick
        system_.SetSolverType(chrono::ChSolver::Type::BARZILAIBORWEIN);
        system_.GetSolver()->AsIterative()->SetMaxIterations(40);

        auto material = chrono_types::make_shared<chrono::ChContactMaterialNSC>();
        material->SetFriction(0.8f);
        material->SetRestitution(0.1f);
        auto ground = chrono_types::make_shared<chrono::ChBodyEasyBox>(20.0, 20.0, 0.1, 1000.0,
                                                                       false, true, material);
        ground->SetPos(chrono::ChVector3d(0.0, 0.0, -0.05));
        ground->SetFixed(true);
        system_.Add(ground);

        robot_.load(std::filesystem::path(ROBOT_FILE));
        if (!robot_.get_camera())
        {
            throw std::runtime_error("The robot file " + ROBOT_FILE + " has no front-camera");
        }
        robot_.set_world_pose(start.x, start.y, start.yaw);
        robot_.add_to_sys(system_);
    }

    void step(const Command &command)
    {
        robot_.set_velocity(command.v, command.omega);
        robot_.update(STEP_SIZE);
        system_.DoStepDynamics(STEP_SIZE);
    }

    real_t time() const { return system_.GetChTime(); }
    Point position() const { return {robot_.axle_center().x(), robot_.axle_center().y()}; }
    real_t heading() const { return robot_.heading(); }
    const CHRONO_DiffDriveRobot &robot() const { return robot_; }

  private:
    chrono::ChSystemNSC system_;
    CHRONO_DiffDriveRobot robot_;
};

// ---------------------------------------------------------------------------
// The picture
// ---------------------------------------------------------------------------

class Display
{
  public:
    // Shows the picture in a window if show_window, and writes it to the video
    // file record if that is not empty, frame_rate pictures a second
    Display(const FloorImage &floor, bool show_window, const std::string &record, real_t frame_rate)
        : show_window_(show_window), record_(record), frame_rate_(frame_rate)
    {
        scale_ = std::min(1.0, 900.0 / floor.image().cols);
        if (show_window_)
        {
            cv::namedWindow(WINDOW_TITLE, cv::WINDOW_AUTOSIZE);
        }
    }

    // Returns false once the window is closed or Esc is pressed
    bool show(const FloorImage &floor, const World &world, const cv::Mat &view,
              const LineReading &line, const Path &path, const std::vector<Point> &trail,
              const std::string &text)
    {
        cv::Mat picture;
        cv::resize(floor.image(), picture, cv::Size(), scale_, scale_, cv::INTER_AREA);
        if (picture.channels() == 1)
        {
            cv::cvtColor(picture, picture, cv::COLOR_GRAY2BGR);
        }
        auto pixel = [&](const Point &p)
        {
            const auto px = floor.to_pixel(p[0], p[1]);
            return cv::Point(static_cast<int>(std::round(px.x * scale_)),
                             static_cast<int>(std::round(px.y * scale_)));
        };
        auto polygon = [&](const auto &points, const cv::Scalar &colour, int width)
        {
            std::vector<cv::Point> pts;
            for (const auto &p : points)
            {
                pts.push_back(pixel(p));
            }
            cv::polylines(picture, pts, true, colour, width, cv::LINE_AA);
        };

        // The line as the simulator knows it, and where the robot went
        for (const auto &segment : path.segments)
        {
            cv::line(picture, pixel(segment.a), pixel(segment.b), cv::Scalar(80, 180, 80), 1,
                     cv::LINE_AA);
        }
        for (std::size_t i = 1; i < trail.size(); ++i)
        {
            cv::line(picture, pixel(trail[i - 1]), pixel(trail[i]), cv::Scalar(0, 200, 255), 2,
                     cv::LINE_AA);
        }

        // The robot: chassis, wheels and what its camera sees
        const auto &robot = world.robot();
        const auto &data = robot.get_data();
        const Point c = world.position();
        const real_t yaw = world.heading();
        auto body = [&](real_t forward, real_t left) -> Point
        {
            return {c[0] + forward * std::cos(yaw) - left * std::sin(yaw),
                    c[1] + forward * std::sin(yaw) + left * std::cos(yaw)};
        };
        const real_t r = data.wheel_radius;
        const real_t half_axle = 0.5 * data.axle_length;
        polygon(std::array<Point, 4>{body(0.35, 0.15), body(0.35, -0.15), body(-0.15, -0.15),
                                     body(-0.15, 0.15)},
                cv::Scalar(230, 160, 90), 2);
        for (const real_t side : {half_axle, -half_axle})
        {
            cv::line(picture, pixel(body(-r, side)), pixel(body(r, side)), cv::Scalar(60, 60, 60),
                     4, cv::LINE_AA);
        }
        const auto &camera = *robot.get_camera();
        const auto footprint = camera.footprint(c[0], c[1], yaw);
        polygon(footprint, cv::Scalar(220, 200, 0), 1);
        if (line.found)
        {
            cv::circle(picture, pixel(body(line.forward, line.left)), 4, cv::Scalar(0, 0, 255),
                       cv::FILLED, cv::LINE_AA);
        }

        // The camera's image in the top left corner
        // At its own size it fits beside the square, which is centred
        cv::Mat inset = view.clone();
        if (inset.channels() == 1)
        {
            cv::cvtColor(inset, inset, cv::COLOR_GRAY2BGR);
        }
        if (line.found)
        {
            cv::circle(inset, line.pixel, 4, cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_AA);
        }
        cv::rectangle(inset, cv::Rect(0, 0, inset.cols, inset.rows), cv::Scalar(220, 200, 0), 2);
        if (inset.cols + 10 < picture.cols && inset.rows + 40 < picture.rows)
        {
            inset.copyTo(picture(cv::Rect(10, 10, inset.cols, inset.rows)));
            cv::putText(picture, "robot camera", cv::Point(12, inset.rows + 28),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(40, 40, 40), 1, cv::LINE_AA);
        }

        // The numbers along the bottom
        cv::rectangle(picture, cv::Rect(0, picture.rows - 28, picture.cols, 28),
                      cv::Scalar(250, 250, 250), cv::FILLED);
        cv::putText(picture, text, cv::Point(8, picture.rows - 9), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(30, 30, 30), 1, cv::LINE_AA);

        // One picture every 1 / frame_rate_ s of simulation, so the video
        // plays at the speed of the simulation
        if (!record_.empty() && world.time() + 1e-9 >= next_record_)
        {
            next_record_ += 1.0 / frame_rate_;
            if (!writer_.isOpened())
            {
                // MJPG in an .avi file needs no codec beyond OpenCV's own
                const int codec = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
                if (!writer_.open(record_, codec, frame_rate_, picture.size()))
                {
                    throw std::runtime_error("Cannot write the video " + record_);
                }
            }
            writer_.write(picture);
        }
        if (!show_window_)
        {
            return true;
        }

        cv::imshow(WINDOW_TITLE, picture);
        const int key = cv::waitKey(1);

        // The window is only visible a little after the first imshow, so it
        // counts as closed once it has been visible and is not any more
        const bool visible = cv::getWindowProperty(WINDOW_TITLE, cv::WND_PROP_VISIBLE) >= 1.0;
        was_visible_ = was_visible_ || visible;
        return key != 27 && (visible || !was_visible_);
    }

  private:
    bool show_window_;
    std::string record_;
    real_t frame_rate_;
    cv::VideoWriter writer_;
    real_t next_record_{0.0};
    bool was_visible_{false};
    real_t scale_{1.0};
};

// ---------------------------------------------------------------------------
// Running it
// ---------------------------------------------------------------------------

struct Options
{
    std::string server{DEFAULT_SERVER};
    real_t floor_width{DEFAULT_FLOOR_WIDTH};
    bool pi_camera{false};
    bool render{false};
    std::string record;

    std::string url() const
    {
        return server + (pi_camera ? CAMERA_ENDPOINT : SQUARE_PATH_ENDPOINT);
    }
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
        else if (arg == "--pi-camera")
        {
            options.pi_camera = true;
        }
        else if (arg == "--record" && i + 1 < argc)
        {
            options.record = argv[++i];
        }
        else if (arg == "--floor-width" && i + 1 < argc)
        {
            options.floor_width = std::stod(argv[++i]);
        }
        else if (arg == "--help" || arg == "-h" || arg.rfind("--", 0) == 0)
        {
            std::cout
                << "Usage: example_31 [--pi-camera] [--render] [--record FILE] [--floor-width W] "
                   "[server URL]\n"
                << "  Follows the square path of " << SQUARE_PATH_ENDPOINT
                << ", or with --pi-camera\n  the Pi camera at " << CAMERA_ENDPOINT
                << ". The default server is " << DEFAULT_SERVER << ",\n  the default floor width "
                << DEFAULT_FLOOR_WIDTH << " m" << std::endl;
            std::exit(arg == "--help" || arg == "-h" ? 0 : 1);
        }
        else
        {
            options.server = arg;
            while (!options.server.empty() && options.server.back() == '/')
            {
                options.server.pop_back();
            }
        }
    }
    return options;
}

real_t milliseconds_since(std::chrono::steady_clock::time_point t)
{
    return std::chrono::duration<real_t, std::milli>(std::chrono::steady_clock::now() - t).count();
}

// On the square path, wait until the picture switches to the next side, so
// that the robot starts at the beginning of a side with all of its time ahead
std::optional<std::pair<Segment, real_t>> wait_for_next_side(const MJPEG_HTTPCameraBackend &stream,
                                                             real_t floor_width, real_t timeout)
{
    std::optional<Point> first;
    const auto start = std::chrono::steady_clock::now();
    uint_t last_index = 0;
    while (milliseconds_since(start) < 1000.0 * timeout)
    {
        const auto frame = stream.latest_frame();
        if (frame && frame->index != last_index)
        {
            last_index = frame->index;
            if (const auto side = find_segment(FloorImage(frame->image, floor_width)))
            {
                const Point m = side->first.middle();
                if (!first)
                {
                    first = m;
                }
                else if (std::hypot(m[0] - (*first)[0], m[1] - (*first)[1]) > 0.1)
                {
                    return side;
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return std::nullopt;
}

int run(const Options &options)
{
    MJPEG_HTTPCameraBackend::Config config;
    config.url = options.url();
    MJPEG_HTTPCameraBackend stream(config);
    stream.connect();

    std::cout << "Waiting for pictures from " << config.url << " ..." << std::endl;
    if (!stream.wait_for_frame(15.0))
    {
        std::cerr << "No pictures from " << config.url << ". Is the server running?\n"
                  << "Without a Pi, start camera_server.py --synthetic and pass "
                     "http://localhost:8000"
                  << std::endl;
        return 1;
    }

    CameraFrame frame = *stream.latest_frame();
    FloorImage floor(frame.image, options.floor_width);
    std::cout << std::fixed << std::setprecision(3) << "Pictures " << frame.image.cols << " x "
              << frame.image.rows << " pixels, the floor is " << floor.width() << " x "
              << floor.height() << " m\n";

    // Find the line and where to start on it
    Path path;
    Start start{};
    if (options.pi_camera)
    {
        const auto square = find_square(floor);
        if (!square)
        {
            std::cerr << "There is no square in the camera's picture. Draw one with thick dark "
                         "lines and make sure the camera sees all of it"
                      << std::endl;
            return 1;
        }
        path = *square;
        start = start_on_square(path);
        std::cout << "Square: side " << path.segments.front().length() << " m, line "
                  << path.line_width << " m wide, centre (" << path.centre[0] << ", "
                  << path.centre[1] << ")\n";
    }
    else
    {
        std::cout << "Waiting for the next side of the square path ..." << std::endl;
        const auto side = wait_for_next_side(stream, options.floor_width, WAIT_TIME);
        if (!side)
        {
            std::cerr << "The pictures do not show a side of a square that changes. Is "
                      << config.url << " the square path?" << std::endl;
            return 1;
        }
        frame = *stream.latest_frame();
        floor = FloorImage(frame.image, options.floor_width);
        path.line_width = side->second;
        const Segment first = counterclockwise(side->first, path.centre);
        path.add(first);
        start = start_on_side(first);
        std::cout << "Side: " << first.length() << " m long, line " << path.line_width
                  << " m wide\n";
    }

    World world(start);
    const auto &camera = *world.robot().get_camera();
    const real_t half_view_width = 0.5 * camera.config().view_width;
    LineFollower follower(half_view_width, options.pi_camera ? CRUISE_SPEED : SQUARE_PATH_SPEED,
                          options.pi_camera ? LineFollower::WhenLost::SEARCH
                                            : LineFollower::WhenLost::WAIT,
                          options.pi_camera ? LOST_TIME : WAIT_TIME);
    const real_t near_edge = camera.config().offset - 0.5 * camera.config().view_length;
    SquarePathDriver driver(follower, near_edge, 0.5 * path.line_width);

    std::optional<Display> display;
    if (options.render || !options.record.empty())
    {
        display.emplace(floor, options.render, options.record, camera.config().update_rate);
    }

    std::cout << "Start at (" << start.x << ", " << start.y << ") heading "
              << start.yaw * 180.0 / PI << " deg\n\n"
              << "    t [s]   x [m]   y [m]  head [deg]  line left [m]  v [m/s]  w [rad/s]"
                 "  off line [m]  picture age [ms]\n";

    const real_t camera_period = 1.0 / camera.config().update_rate;
    real_t next_image = 0.0;
    real_t next_print = 0.0;
    cv::Mat view;
    LineReading line;
    Command command;
    std::vector<Point> trail;

    // Progress around the centre of the square, and how far from the line the
    // robot drives
    real_t last_angle = std::atan2(start.y - path.centre[1], start.x - path.centre[0]);
    real_t travelled_angle = 0.0;
    real_t error_sum = 0.0;
    real_t max_error = 0.0;
    uint_t n_errors = 0;
    uint_t n_images = 0;
    uint_t n_lost_images = 0;
    real_t frame_age_sum = 0.0;
    real_t frame_age = 0.0; // of the picture in the latest camera image [ms]
    bool completed = false;
    bool stopped = false;

    const auto wall_start = std::chrono::steady_clock::now();
    while (world.time() < TIME_LIMIT)
    {
        // The camera takes an image of the latest picture of the floor
        if (world.time() >= next_image)
        {
            next_image += camera_period;
            if (const auto latest = stream.latest_frame(); latest && latest->index != frame.index)
            {
                frame = *latest;
                floor = FloorImage(frame.image, options.floor_width);
                // On the square path, remember each side as it appears
                if (!options.pi_camera)
                {
                    if (const auto side = find_segment(floor))
                    {
                        path.add(counterclockwise(side->first, path.centre));
                    }
                }
            }
            frame_age = milliseconds_since(frame.received_at);
            frame_age_sum += frame_age;
            view = world.robot().camera_view(floor);
            line = read_line(view, camera);
            command =
                options.pi_camera
                    ? follower.command(line, camera_period)
                    : driver.command(line, world.position(), world.heading(),
                                     nearest_side_heading(path, world.position()), camera_period);
            ++n_images;
            n_lost_images += line.found ? 0 : 1;

            if (options.pi_camera ? follower.has_given_up() : driver.has_given_up())
            {
                std::cout << "\nThe robot saw no line for "
                          << (options.pi_camera ? follower.give_up_after() : driver.give_up_after())
                          << " s and stops" << std::endl;
                break;
            }
            if (!stream.is_alive())
            {
                command = Command{};
            }
        }

        world.step(command);

        const Point p = world.position();
        const real_t angle = std::atan2(p[1] - path.centre[1], p[0] - path.centre[0]);
        travelled_angle += std::remainder(angle - last_angle, 2.0 * PI);
        last_angle = angle;
        const real_t error = path.distance(p);
        error_sum += error;
        max_error = std::max(max_error, error);
        ++n_errors;
        if (trail.empty() || std::hypot(p[0] - trail.back()[0], p[1] - trail.back()[1]) > 0.01)
        {
            trail.push_back(p);
        }

        if (world.time() >= next_print)
        {
            next_print += PRINT_PERIOD;
            std::cout << std::setw(9) << std::setprecision(2) << world.time()
                      << std::setprecision(3) << std::setw(8) << p[0] << std::setw(8) << p[1]
                      << std::setw(12) << std::setprecision(1) << world.heading() * 180.0 / PI
                      << std::setprecision(3) << std::setw(15)
                      << (line.found ? std::to_string(line.left).substr(0, 6) : "lost")
                      << std::setw(9) << command.v << std::setw(11) << command.omega
                      << std::setw(14) << error << std::setw(18) << std::setprecision(0)
                      << frame_age << std::setprecision(3) << std::endl;
        }

        if (display && world.time() + 1e-9 >= next_image)
        {
            std::ostringstream text;
            text << std::fixed << std::setprecision(1) << "t " << world.time() << " s   lap "
                 << 100.0 * std::clamp(travelled_angle / (2.0 * PI), 0.0, 1.0) << " %   v "
                 << std::setprecision(2) << command.v << " m/s   off line " << error
                 << " m   stream " << std::setprecision(0) << stream.frame_rate() << " fps";
            if (!display->show(floor, world, view, line, path, trail, text.str()))
            {
                stopped = true;
                break;
            }
        }

        if (travelled_angle >= 2.0 * PI)
        {
            completed = true;
            break;
        }

        // The pictures arrive in real time, so the simulation keeps to real time
        const real_t ahead =
            world.time() -
            std::chrono::duration<real_t>(std::chrono::steady_clock::now() - wall_start).count();
        if (ahead > 0.0)
        {
            std::this_thread::sleep_for(std::chrono::duration<real_t>(ahead));
        }
    }

    const real_t wall_time =
        std::chrono::duration<real_t>(std::chrono::steady_clock::now() - wall_start).count();
    std::cout << std::setprecision(3) << "\n"
              << (completed ? "The robot went once around the square"
                  : stopped ? "Stopped"
                            : "The robot did not get around the square")
              << " in " << world.time() << " s of simulation, " << wall_time << " s of wall time\n"
              << "Distance of the axle centre from the line: mean " << error_sum / n_errors
              << " m, largest " << max_error << " m (the line is " << path.line_width
              << " m wide)\n"
              << "Camera images: " << n_images << ", without the line in them " << n_lost_images
              << "\n"
              << "Stream: " << std::setprecision(1) << stream.frame_rate()
              << " frames/s, mean age of the picture used " << frame_age_sum / n_images
              << " ms, reconnections " << stream.n_reconnects() << std::endl;

    stream.disconnect();
    return completed ? 0 : 1;
}

} // namespace example_31

int main(int argc, char **argv)
{
    try
    {
        return example_31::run(example_31::parse(argc, argv));
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}

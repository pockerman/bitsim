// Example 31: A line following robot that sees through a Raspberry Pi camera
//
// A Raspberry Pi 5 with a camera looks down at a sheet of paper with a square
// drawn on it and streams what it sees over HTTP. Its picture is the floor of
// the simulated world. The differential drive robot of example 14 carries a
// camera that looks down at the floor just ahead of it; it sees a patch of the
// latest picture, finds the line in it and steers to follow it around the
// square. Move the paper, or draw on it, and the robot sees that.
//
// Start the camera server first, on the Pi or, without a Pi, on this computer:
//
//     python3 camera_server.py               # on the Pi
//     python3 camera_server.py --synthetic   # anywhere, draws a square
//
// Usage: example_31 [--render] [--floor-width W] [stream URL]
//
// The default URL is http://raspberrypi.local:8000/stream. --floor-width is
// how wide, in metres, the floor that the picture covers is; 3 by default.

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

const std::string DEFAULT_URL = "http://raspberrypi.local:8000/stream";
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
const real_t LOST_TIME = 5.0;          // [s] with no line before giving up
const real_t MIN_LINE_COVERAGE = 0.01; // of the camera image
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
    return reading;
}

// ---------------------------------------------------------------------------
// The square on the paper
// ---------------------------------------------------------------------------

// The centre line of the square, in world coordinates, corners in
// counterclockwise order
struct Square
{
    std::array<Point, 4> corners;
    Point centre;
    real_t side{0.0};
    real_t line_width{0.0};

    // How far a point is from the line
    real_t distance(const Point &p) const
    {
        real_t best = std::numeric_limits<real_t>::max();
        for (uint_t i = 0; i < 4; ++i)
        {
            const Point &a = corners[i];
            const Point &b = corners[(i + 1) % 4];
            const real_t dx = b[0] - a[0];
            const real_t dy = b[1] - a[1];
            const real_t s = std::clamp(
                ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / (dx * dx + dy * dy), 0.0, 1.0);
            best = std::min(best, std::hypot(p[0] - a[0] - s * dx, p[1] - a[1] - s * dy));
        }
        return best;
    }
};

// Find the square: the largest dark outline with a hole in it. The centre
// line is halfway between its outer and inner edges
std::optional<Square> find_square(const FloorImage &floor)
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
    Square square;
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
        square.corners[i] = by_angle[i].second;
    }
    const real_t mpp = floor.metres_per_pixel();
    square.side = 0.5 * (centre_line.size.width + centre_line.size.height) * mpp;
    square.line_width =
        0.25 * ((a.size.width - b.size.width) + (a.size.height - b.size.height)) * mpp;
    return square;
}

// Where the robot starts: the middle of the bottom side, facing along it so
// that the inside of the square is on its left
struct Start
{
    real_t x;
    real_t y;
    real_t yaw;
};

Start start_on(const Square &square)
{
    uint_t bottom = 0;
    real_t lowest = std::numeric_limits<real_t>::max();
    for (uint_t i = 0; i < 4; ++i)
    {
        const real_t y = 0.5 * (square.corners[i][1] + square.corners[(i + 1) % 4][1]);
        if (y < lowest)
        {
            lowest = y;
            bottom = i;
        }
    }
    const Point &a = square.corners[bottom];
    const Point &b = square.corners[(bottom + 1) % 4];
    return {0.5 * (a[0] + b[0]), 0.5 * (a[1] + b[1]), std::atan2(b[1] - a[1], b[0] - a[0])};
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
// the robot turns. Slow down in sharp turns, and look for the line by turning
// towards where it was last seen when the camera loses it
class LineFollower
{
  public:
    explicit LineFollower(real_t half_view_width) : half_view_width_(half_view_width) {}

    Command command(const LineReading &line, real_t dt)
    {
        if (!line.found)
        {
            lost_for_ += dt;
            return {0.0, last_left_ >= 0.0 ? SEARCH_TURN_RATE : -SEARCH_TURN_RATE};
        }
        lost_for_ = 0.0;
        last_left_ = line.left;

        const real_t curvature =
            2.0 * line.left / (line.forward * line.forward + line.left * line.left);
        const real_t sideways = std::min(std::abs(line.left) / half_view_width_, 1.0);
        const real_t v = CRUISE_SPEED * std::max(1.0 - sideways, MIN_SPEED_RATIO);
        return {v, std::clamp(v * curvature, -MAX_TURN_RATE, MAX_TURN_RATE)};
    }

    bool is_lost() const { return lost_for_ > LOST_TIME; }

  private:
    real_t half_view_width_;
    real_t last_left_{0.0};
    real_t lost_for_{0.0};
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
    Display(const FloorImage &floor, const Square &square) : square_(square)
    {
        scale_ = std::min(1.0, 900.0 / floor.image().cols);
        cv::namedWindow(WINDOW_TITLE, cv::WINDOW_AUTOSIZE);
    }

    // Returns false once the window is closed or Esc is pressed
    bool show(const FloorImage &floor, const World &world, const cv::Mat &view,
              const LineReading &line, const std::vector<Point> &trail, const std::string &text)
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

        // The square found in the first picture, and where the robot went
        polygon(square_.corners, cv::Scalar(80, 180, 80), 1);
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
        cv::Mat inset;
        cv::resize(view, inset, cv::Size(), 1.5, 1.5, cv::INTER_NEAREST);
        if (inset.channels() == 1)
        {
            cv::cvtColor(inset, inset, cv::COLOR_GRAY2BGR);
        }
        if (line.found)
        {
            cv::circle(inset, line.pixel * 1.5, 5, cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_AA);
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

        cv::imshow(WINDOW_TITLE, picture);
        const int key = cv::waitKey(1);

        // The window is only visible a little after the first imshow, so it
        // counts as closed once it has been visible and is not any more
        const bool visible = cv::getWindowProperty(WINDOW_TITLE, cv::WND_PROP_VISIBLE) >= 1.0;
        was_visible_ = was_visible_ || visible;
        return key != 27 && (visible || !was_visible_);
    }

  private:
    Square square_;
    bool was_visible_{false};
    real_t scale_{1.0};
};

// ---------------------------------------------------------------------------
// Running it
// ---------------------------------------------------------------------------

struct Options
{
    std::string url{DEFAULT_URL};
    real_t floor_width{DEFAULT_FLOOR_WIDTH};
    bool render{false};
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
        else if (arg == "--floor-width" && i + 1 < argc)
        {
            options.floor_width = std::stod(argv[++i]);
        }
        else if (arg == "--help" || arg == "-h" || arg.rfind("--", 0) == 0)
        {
            std::cout << "Usage: example_31 [--render] [--floor-width W] [stream URL]\n"
                      << "  default URL " << DEFAULT_URL << ", default floor width "
                      << DEFAULT_FLOOR_WIDTH << " m" << std::endl;
            std::exit(arg == "--help" || arg == "-h" ? 0 : 1);
        }
        else
        {
            options.url = arg;
        }
    }
    return options;
}

real_t milliseconds_since(std::chrono::steady_clock::time_point t)
{
    return std::chrono::duration<real_t, std::milli>(std::chrono::steady_clock::now() - t).count();
}

int run(const Options &options)
{
    MJPEG_HTTPCameraBackend::Config config;
    config.url = options.url;
    MJPEG_HTTPCameraBackend stream(config);
    stream.connect();

    std::cout << "Waiting for the camera at " << options.url << " ..." << std::endl;
    if (!stream.wait_for_frame(15.0))
    {
        std::cerr << "No frames from " << options.url << ". Is the camera server running?\n"
                  << "Start it with: python3 camera_server.py [--synthetic]" << std::endl;
        return 1;
    }

    CameraFrame frame = *stream.latest_frame();
    FloorImage floor(frame.image, options.floor_width);
    const auto square = find_square(floor);
    if (!square)
    {
        std::cerr << "There is no square in the camera's picture. Draw one with thick dark lines "
                     "and make sure the camera sees all of it"
                  << std::endl;
        return 1;
    }

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Camera picture " << frame.image.cols << " x " << frame.image.rows
              << " pixels, the floor is " << floor.width() << " x " << floor.height() << " m\n"
              << "Square: side " << square->side << " m, line " << square->line_width
              << " m wide, centre (" << square->centre[0] << ", " << square->centre[1] << ")\n";

    const Start start = start_on(*square);
    World world(start);
    const auto &camera = *world.robot().get_camera();
    LineFollower follower(0.5 * camera.config().view_width);

    std::optional<Display> display;
    if (options.render)
    {
        display.emplace(floor, *square);
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

    // Progress around the square, and how far from the line the robot drives
    const Point start_offset{start.x - square->centre[0], start.y - square->centre[1]};
    real_t last_angle = std::atan2(start_offset[1], start_offset[0]);
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
            }
            frame_age = milliseconds_since(frame.received_at);
            frame_age_sum += frame_age;
            view = world.robot().camera_view(floor);
            line = read_line(view, camera);
            command = follower.command(line, camera_period);
            ++n_images;
            n_lost_images += line.found ? 0 : 1;

            if (follower.is_lost())
            {
                std::cout << "\nThe robot lost the line for " << LOST_TIME << " s and stops"
                          << std::endl;
                break;
            }
            if (!stream.is_alive())
            {
                command = Command{};
            }
        }

        world.step(command);

        const Point p = world.position();
        const real_t angle = std::atan2(p[1] - square->centre[1], p[0] - square->centre[0]);
        travelled_angle += std::remainder(angle - last_angle, 2.0 * PI);
        last_angle = angle;
        const real_t error = square->distance(p);
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
                 << 100.0 * std::min(travelled_angle / (2.0 * PI), 1.0) << " %   v "
                 << std::setprecision(2) << command.v << " m/s   off line " << error
                 << " m   stream " << std::setprecision(0) << stream.frame_rate() << " fps";
            if (!display->show(floor, world, view, line, trail, text.str()))
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

        // The camera is real, so the simulation keeps to real time
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
              << " m, largest " << max_error << " m (the line is " << square->line_width
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

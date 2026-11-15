#include "bitrl/bitrl_config.h"
#include "bitrl/bitrl_consts.h"
#include "bitrl/bitrl_types.h"
#include "bitrl/sensors/downward_camera.h"

#include <gtest/gtest.h>
#include <opencv2/imgproc.hpp>

#include <array>
#include <cmath>
#include <stdexcept>

namespace
{

using bitrl::real_t;
using bitrl::sensors::DownwardCamera;
using bitrl::sensors::FloorImage;

const real_t PI = bitrl::consts::maths::PI;

// A white floor 4 m x 3 m, 1 cm per pixel, with a black disc of radius 2 cm
// centred on the pixel nearest to (x, y). dot is where the disc really is
FloorImage floor_with_dot(real_t x, real_t y, std::array<real_t, 2> &dot)
{
    cv::Mat image(300, 400, CV_8UC1, cv::Scalar(255));
    const FloorImage floor(image, 4.0);
    const auto pixel = floor.to_pixel(x, y);
    const cv::Point centre(static_cast<int>(std::round(pixel.x)),
                           static_cast<int>(std::round(pixel.y)));
    cv::circle(image, centre, 2, cv::Scalar(0), cv::FILLED);
    dot = floor.to_world(cv::Point2d(centre));
    return floor;
}

// The centre of the dark pixels of an image
cv::Point2d dark_centre(const cv::Mat &image)
{
    cv::Mat dark = image < 128;
    const auto m = cv::moments(dark, true);
    EXPECT_GT(m.m00, 0.0) << "the camera does not see the dot";
    return {m.m10 / m.m00, m.m01 / m.m00};
}

} // namespace

TEST(TestFloorImage, MapsTheCentreAndCornersOfTheFloor)
{
    const FloorImage floor(cv::Mat(300, 400, CV_8UC1, cv::Scalar(255)), 4.0);
    EXPECT_DOUBLE_EQ(floor.metres_per_pixel(), 0.01);
    EXPECT_DOUBLE_EQ(floor.height(), 3.0);

    // The origin is in the middle, and the top of the image is the largest y
    const auto centre = floor.to_pixel(0.0, 0.0);
    EXPECT_NEAR(centre.x, 199.5, 1e-9);
    EXPECT_NEAR(centre.y, 149.5, 1e-9);
    const auto top_left = floor.to_pixel(-2.0, 1.5);
    EXPECT_NEAR(top_left.x, -0.5, 1e-9);
    EXPECT_NEAR(top_left.y, -0.5, 1e-9);

    const auto back = floor.to_world(floor.to_pixel(0.7, -0.4));
    EXPECT_NEAR(back[0], 0.7, 1e-12);
    EXPECT_NEAR(back[1], -0.4, 1e-12);
}

TEST(TestDownwardCamera, SeesWhatIsAheadInTheMiddleOfItsImage)
{
    // The dot is right where the middle of the view is
    DownwardCamera camera;
    std::array<real_t, 2> dot;
    const auto floor = floor_with_dot(camera.config().offset, 0.0, dot);
    const cv::Mat view = camera.view(floor, 0.0, 0.0, 0.0);

    ASSERT_EQ(view.cols, 160);
    ASSERT_EQ(view.rows, 120);
    const auto centre = dark_centre(view);
    EXPECT_NEAR(centre.x, 79.5, 2.5);
    EXPECT_NEAR(centre.y, 59.5, 2.5);

    // To a camera pixel, 2.5 mm, where the dot really is
    const auto where = camera.to_robot(centre.x, centre.y);
    EXPECT_NEAR(where[0], dot[0], 0.003);
    EXPECT_NEAR(where[1], dot[1], 0.003);
}

TEST(TestDownwardCamera, ShowsTheLeftOnTheLeftAndTheFarSideAtTheTop)
{
    DownwardCamera camera;
    const real_t ahead = camera.config().offset + 0.1;
    const real_t left = 0.1;

    // The robot faces +y, so its left is -x
    std::array<real_t, 2> dot;
    const auto floor = floor_with_dot(-left, ahead, dot);
    const auto centre = dark_centre(camera.view(floor, 0.0, 0.0, PI / 2.0));

    const auto where = camera.to_robot(centre.x, centre.y);
    EXPECT_NEAR(where[0], dot[1], 0.003);
    EXPECT_NEAR(where[1], -dot[0], 0.003);
    EXPECT_LT(centre.x, 79.5);
    EXPECT_LT(centre.y, 59.5);
}

TEST(TestDownwardCamera, FootprintSurroundsTheView)
{
    DownwardCamera camera;
    const auto corners = camera.footprint(1.0, 2.0, 0.0);
    const auto &c = camera.config();

    // far left, far right, near right, near left
    EXPECT_NEAR(corners[0][0], 1.0 + c.offset + c.view_length / 2.0, 1e-12);
    EXPECT_NEAR(corners[0][1], 2.0 + c.view_width / 2.0, 1e-12);
    EXPECT_NEAR(corners[2][0], 1.0 + c.offset - c.view_length / 2.0, 1e-12);
    EXPECT_NEAR(corners[2][1], 2.0 - c.view_width / 2.0, 1e-12);
}

TEST(TestDownwardCamera, IsWhiteBeyondTheFloor)
{
    DownwardCamera camera;
    std::array<real_t, 2> dot;
    const auto floor = floor_with_dot(0.0, 0.0, dot);
    const cv::Mat view = camera.view(floor, 10.0, 10.0, 0.0);
    EXPECT_EQ(cv::countNonZero(view < 255), 0);
}

TEST(TestDownwardCamera, ReadsItsConfigAndRejectsBadOnes)
{
    DownwardCamera::Config config;
    config.read(nlohmann::json{{"offset", 0.5}, {"image_width", 64}});
    EXPECT_DOUBLE_EQ(config.offset, 0.5);
    EXPECT_EQ(config.image_width, 64u);
    EXPECT_DOUBLE_EQ(config.view_length, DownwardCamera::Config().view_length);

    config.view_width = 0.0;
    EXPECT_THROW(DownwardCamera{config}, std::logic_error);
    EXPECT_THROW(FloorImage(cv::Mat(), 1.0), std::logic_error);
}

#ifndef DOWNWARD_CAMERA_H
#define DOWNWARD_CAMERA_H

#include "bitrl/bitrl_config.h"

#ifdef BITRL_OPENCV

#include "bitrl/bitrl_types.h"
#include "bitrl/extern/nlohmann/json/json.hpp"

#include <opencv2/core.hpp>

#include <array>
#include <functional>
#include <string>

namespace bitrl
{
namespace sensors
{

///
/// @brief An image of the floor, centred on the world origin
///
/// @details The image covers width() x height() metres of the floor. Its
/// centre is at the origin, its columns grow with x and its rows grow against
/// y, so the top of the image is the side of the floor with the largest y.
///
class FloorImage
{
  public:
    ///
    /// @brief Constructor
    /// @param image The image of the floor
    /// @param width The width of the floor the image covers, along x [m]
    ///
    FloorImage(const cv::Mat &image, real_t width);

    ///
    /// @brief Draw a floor: a grey picture of width x height metres of paper, with ink wherever
    /// is_ink(x, y) is true
    ///
    /// @param width The width of the floor, along x [m]
    /// @param height The height of the floor, along y [m]
    /// @param metres_per_pixel The size of a pixel [m]
    /// @param is_ink Whether the point (x, y) of the floor is inked
    /// @param paper The brightness of the paper, 0 to 255
    /// @param ink The brightness of the ink
    ///
    static FloorImage draw(real_t width, real_t height, real_t metres_per_pixel,
                           const std::function<bool(real_t, real_t)> &is_ink, uchar paper = 235,
                           uchar ink = 25);

    ///
    /// @brief Write the image as a colour PNG, for a texture: Irrlicht reads colour pictures more
    /// reliably than grey ones. Throws if the file cannot be written
    ///
    void write_texture(const std::string &path) const;

    const cv::Mat &image() const noexcept { return image_; }

    ///
    /// @brief The width of the floor, along x [m]
    ///
    real_t width() const noexcept { return width_; }

    ///
    /// @brief The height of the floor, along y [m]
    ///
    real_t height() const noexcept { return metres_per_pixel_ * image_.rows; }

    ///
    /// @brief The size of a pixel [m]
    ///
    real_t metres_per_pixel() const noexcept { return metres_per_pixel_; }

    ///
    /// @brief The pixel, with fractions, at the given point of the floor
    ///
    cv::Point2d to_pixel(real_t x, real_t y) const;

    ///
    /// @brief The point of the floor at the given pixel, with fractions
    ///
    std::array<real_t, 2> to_world(const cv::Point2d &pixel) const;

    ///
    /// @brief How bright the floor is at a point, from 0 for black to 1 for
    /// white: the brightness of the pixel there. Outside the image the floor is
    /// white. An infrared element looking at the floor reads this, see
    /// backends::CHRONO_InfraredBackend
    ///
    real_t reflectance(real_t x, real_t y) const;

  private:
    cv::Mat image_;
    real_t width_;
    real_t metres_per_pixel_;
};

///
/// @brief A camera on a robot that looks straight down at the floor ahead
///
/// @details The camera sees a rectangle of the floor centred offset metres
/// ahead of the robot's reference point, usually the centre of its axle. The
/// rows of its image run from the far edge of that rectangle, row 0, to the
/// near edge, and the columns from the robot's left, column 0, to its right,
/// as a camera that faces forward and tilts down would show them.
///
class DownwardCamera
{
  public:
    ///
    /// @brief The camera's mounting and image
    ///
    struct Config
    {
        ///
        /// @brief How far ahead of the reference point the centre of the
        /// view is [m]
        ///
        real_t offset{0.35};

        ///
        /// @brief The length of the floor the camera sees along the robot's
        /// heading [m]
        ///
        real_t view_length{0.3};

        ///
        /// @brief The width of the floor the camera sees across the robot [m]
        ///
        real_t view_width{0.4};

        ///
        /// @brief The size of the camera's image [pixels]
        ///
        uint_t image_width{160};
        uint_t image_height{120};

        ///
        /// @brief How many images the camera takes per second [1/s]
        ///
        real_t update_rate{20.0};

        ///
        /// @brief Read the values that the json gives
        ///
        void read(const nlohmann::json &j);
    };

    ///
    /// @brief A camera with the default Config
    ///
    DownwardCamera();

    explicit DownwardCamera(const Config &config);

    const Config &config() const noexcept { return config_; }

    ///
    /// @brief What the camera sees with the robot at (x, y) heading yaw
    /// @details Floor outside the image is white
    ///
    cv::Mat view(const FloorImage &floor, real_t x, real_t y, real_t yaw) const;

    ///
    /// @brief Where a pixel of the camera's image is relative to the robot
    /// @return {forward, left}: metres ahead of the reference point and metres
    /// to its left
    ///
    std::array<real_t, 2> to_robot(real_t column, real_t row) const;

    ///
    /// @brief The corners of the floor the camera sees, in world coordinates:
    /// far left, far right, near right, near left
    ///
    std::array<std::array<real_t, 2>, 4> footprint(real_t x, real_t y, real_t yaw) const;

  private:
    Config config_;
};

} // namespace sensors
} // namespace bitrl

#endif
#endif // DOWNWARD_CAMERA_H

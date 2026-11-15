#include "bitrl/bitrl_config.h"

#ifdef BITRL_OPENCV

#include "bitrl/sensors/downward_camera.h"

#include <opencv2/imgproc.hpp>

#include <cmath>
#include <stdexcept>

namespace bitrl
{
namespace sensors
{

FloorImage::FloorImage(const cv::Mat &image, real_t width)
    : image_(image), width_(width), metres_per_pixel_(0.0)
{
    if (image_.empty())
    {
        throw std::logic_error("FloorImage needs an image");
    }
    if (width_ <= 0.0)
    {
        throw std::logic_error("FloorImage needs a positive width");
    }
    metres_per_pixel_ = width_ / image_.cols;
}

cv::Point2d FloorImage::to_pixel(real_t x, real_t y) const
{
    // Pixel centres are at whole numbers
    return {(x + 0.5 * width()) / metres_per_pixel_ - 0.5,
            (0.5 * height() - y) / metres_per_pixel_ - 0.5};
}

std::array<real_t, 2> FloorImage::to_world(const cv::Point2d &pixel) const
{
    return {(pixel.x + 0.5) * metres_per_pixel_ - 0.5 * width(),
            0.5 * height() - (pixel.y + 0.5) * metres_per_pixel_};
}

void DownwardCamera::Config::read(const nlohmann::json &j)
{
    offset = j.value("offset", offset);
    view_length = j.value("view_length", view_length);
    view_width = j.value("view_width", view_width);
    image_width = j.value("image_width", image_width);
    image_height = j.value("image_height", image_height);
    update_rate = j.value("update_rate", update_rate);
}

DownwardCamera::DownwardCamera() : DownwardCamera(Config()) {}

DownwardCamera::DownwardCamera(const Config &config) : config_(config)
{
    if (config_.view_length <= 0.0 || config_.view_width <= 0.0)
    {
        throw std::logic_error("DownwardCamera needs a view of positive size");
    }
    if (config_.image_width == 0 || config_.image_height == 0)
    {
        throw std::logic_error("DownwardCamera needs an image of positive size");
    }
    if (config_.update_rate <= 0.0)
    {
        throw std::logic_error("DownwardCamera needs a positive update rate");
    }
}

std::array<real_t, 2> DownwardCamera::to_robot(real_t column, real_t row) const
{
    const auto &c = config_;
    const real_t forward = c.offset + c.view_length * (0.5 - (row + 0.5) / c.image_height);
    const real_t left = c.view_width * (0.5 - (column + 0.5) / c.image_width);
    return {forward, left};
}

std::array<std::array<real_t, 2>, 4> DownwardCamera::footprint(real_t x, real_t y, real_t yaw) const
{
    const auto &c = config_;
    const real_t cos_yaw = std::cos(yaw);
    const real_t sin_yaw = std::sin(yaw);
    const real_t far = c.offset + 0.5 * c.view_length;
    const real_t near = c.offset - 0.5 * c.view_length;
    const real_t half = 0.5 * c.view_width;

    auto world = [&](real_t forward, real_t left) -> std::array<real_t, 2> {
        return {x + forward * cos_yaw - left * sin_yaw, y + forward * sin_yaw + left * cos_yaw};
    };
    return {world(far, half), world(far, -half), world(near, -half), world(near, half)};
}

cv::Mat DownwardCamera::view(const FloorImage &floor, real_t x, real_t y, real_t yaw) const
{
    const real_t cos_yaw = std::cos(yaw);
    const real_t sin_yaw = std::sin(yaw);

    // The floor pixel that a camera pixel sees. It is an affine function of
    // the camera pixel, so three points give the whole map
    auto floor_pixel = [&](real_t column, real_t row)
    {
        const auto [forward, left] = to_robot(column, row);
        return floor.to_pixel(x + forward * cos_yaw - left * sin_yaw,
                              y + forward * sin_yaw + left * cos_yaw);
    };
    const cv::Point2d origin = floor_pixel(0.0, 0.0);
    const cv::Point2d along_row = floor_pixel(1.0, 0.0) - origin;
    const cv::Point2d along_column = floor_pixel(0.0, 1.0) - origin;

    const cv::Matx23d camera_to_floor(along_row.x, along_column.x, origin.x, along_row.y,
                                      along_column.y, origin.y);

    cv::Mat image;
    cv::warpAffine(
        floor.image(), image, camera_to_floor,
        cv::Size(static_cast<int>(config_.image_width), static_cast<int>(config_.image_height)),
        cv::INTER_LINEAR | cv::WARP_INVERSE_MAP, cv::BORDER_CONSTANT, cv::Scalar::all(255));
    return image;
}

} // namespace sensors
} // namespace bitrl

#endif

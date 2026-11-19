#include "bitrl/bitrl_config.h"

#ifdef BITRL_OPENCV

#include "bitrl/sensors/downward_camera.h"

#include <opencv2/imgcodecs.hpp>
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

FloorImage FloorImage::draw(real_t width, real_t height, real_t metres_per_pixel,
                            const std::function<bool(real_t, real_t)> &is_ink, uchar paper,
                            uchar ink)
{
    const int cols = static_cast<int>(std::round(width / metres_per_pixel));
    const int rows = static_cast<int>(std::round(height / metres_per_pixel));
    cv::Mat image(rows, cols, CV_8UC1, cv::Scalar(paper));
    FloorImage floor(image, width);
    for (int r = 0; r < rows; ++r)
    {
        for (int c = 0; c < cols; ++c)
        {
            const auto p = floor.to_world(cv::Point2d(c, r));
            if (is_ink(p[0], p[1]))
            {
                image.at<uchar>(r, c) = ink;
            }
        }
    }
    return floor;
}

void FloorImage::write_texture(const std::string &path) const
{
    cv::Mat colour;
    if (image_.channels() == 1)
    {
        cv::cvtColor(image_, colour, cv::COLOR_GRAY2BGR);
    }
    else
    {
        colour = image_;
    }
    if (!cv::imwrite(path, colour))
    {
        throw std::runtime_error("FloorImage: cannot write the texture " + path);
    }
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

real_t FloorImage::reflectance(real_t x, real_t y) const
{
    const auto pixel = to_pixel(x, y);
    const int column = static_cast<int>(std::lround(pixel.x));
    const int row = static_cast<int>(std::lround(pixel.y));
    if (column < 0 || row < 0 || column >= image_.cols || row >= image_.rows)
    {
        return 1.0;
    }

    if (image_.depth() != CV_8U || (image_.channels() != 1 && image_.channels() != 3))
    {
        throw std::logic_error("FloorImage::reflectance needs an 8 bit grey or BGR image");
    }
    if (image_.channels() == 1)
    {
        return image_.at<uchar>(row, column) / 255.0;
    }
    // The luma of the colour, as cv::COLOR_BGR2GRAY has it
    const auto &bgr = image_.at<cv::Vec3b>(row, column);
    return (0.114 * bgr[0] + 0.587 * bgr[1] + 0.299 * bgr[2]) / 255.0;
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

std::vector<std::array<real_t, 2>> DownwardCamera::line_points(const cv::Mat &view, uint_t n_bands,
                                                               real_t dark_ratio,
                                                               real_t min_share) const
{
    cv::Mat gray;
    if (view.channels() == 3)
    {
        cv::cvtColor(view, gray, cv::COLOR_BGR2GRAY);
    }
    else
    {
        gray = view;
    }
    double brightest = 0.0;
    cv::minMaxLoc(gray, nullptr, &brightest);
    cv::Mat dark;
    cv::threshold(gray, dark, dark_ratio * brightest, 255, cv::THRESH_BINARY_INV);

    std::vector<std::array<real_t, 2>> points;
    if (n_bands == 0)
    {
        return points;
    }
    const int rows = dark.rows / static_cast<int>(n_bands);
    for (uint_t b = 0; b < n_bands; ++b)
    {
        const cv::Rect band(0, static_cast<int>(b) * rows, dark.cols, rows);
        const cv::Moments m = cv::moments(dark(band), true);
        if (m.m00 < min_share * band.area())
        {
            continue;
        }
        points.push_back(to_robot(m.m10 / m.m00, band.y + m.m01 / m.m00));
    }
    return points;
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

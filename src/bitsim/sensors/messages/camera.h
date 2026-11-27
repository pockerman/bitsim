//
// Created by alex on 11/23/25.
//

#ifndef BITSIM_SENSORS_MESSAGES_CAMERA_H
#define BITSIM_SENSORS_MESSAGES_CAMERA_H

#include <opencv2/opencv.hpp>
#include <optional>
#include <string>

namespace bitsim
{
namespace sensors
{
struct CameraMessage
{
    cv::Mat image;
    static std::optional<CameraMessage> parse(const std::string &base64jpeg);
};
} // namespace sensors
} // namespace bitsim

#endif // BITSIM_SENSORS_MESSAGES_CAMERA_H

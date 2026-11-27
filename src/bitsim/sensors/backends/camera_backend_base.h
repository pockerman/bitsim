#ifndef BITSIM_SENSORS_BACKENDS_CAMERA_BACKEND_BASE_H
#define BITSIM_SENSORS_BACKENDS_CAMERA_BACKEND_BASE_H

#include "bitsim/bitsim_config.h"

#ifdef BITSIM_OPENCV

#include "bitsim/bitsim_types.h"

#include <opencv2/core.hpp>

#include <chrono>
#include <optional>
#include <string>

namespace bitsim
{
namespace sensors::backends
{

///
/// @brief A frame delivered by a camera backend
///
struct CameraFrame
{
    ///
    /// @brief The image, in BGR order as OpenCV uses
    ///
    cv::Mat image;

    ///
    /// @brief How many frames the backend has delivered, this one included
    ///
    uint_t index{0};

    ///
    /// @brief When the backend received the frame
    ///
    std::chrono::steady_clock::time_point received_at{};
};

///
/// @brief Base class for the sources of camera frames
///
class CameraBackendBase
{
  public:
    virtual ~CameraBackendBase() = default;

    ///
    /// @brief Start receiving frames
    ///
    virtual void connect() = 0;

    ///
    /// @brief Stop receiving frames
    ///
    virtual void disconnect() = 0;

    ///
    /// @brief The latest frame, if one has been received
    ///
    virtual std::optional<CameraFrame> latest_frame() const = 0;

    ///
    /// @brief Whether frames are arriving
    ///
    virtual bool is_alive() const = 0;

    ///
    /// @brief The type of the backend
    ///
    const std::string &backend_type_str() const noexcept { return backend_type_; }

  protected:
    explicit CameraBackendBase(const std::string &backend_type) : backend_type_(backend_type) {}

  private:
    std::string backend_type_;
};

} // namespace sensors::backends
} // namespace bitsim

#endif
#endif // BITSIM_SENSORS_BACKENDS_CAMERA_BACKEND_BASE_H

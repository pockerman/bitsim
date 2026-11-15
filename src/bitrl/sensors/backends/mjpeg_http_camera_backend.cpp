#include "bitrl/bitrl_config.h"

#ifdef BITRL_OPENCV

#include "bitrl/sensors/backends/mjpeg_http_camera_backend.h"

#include <opencv2/videoio.hpp>
#include <opencv2/videoio/registry.hpp>

#include <chrono>
#include <stdexcept>
#include <vector>

namespace bitrl
{
namespace sensors::backends
{

namespace
{
using clock_type = std::chrono::steady_clock;

real_t seconds_since(clock_type::time_point t)
{
    return std::chrono::duration<real_t>(clock_type::now() - t).count();
}

// How much a new frame interval changes the average
const real_t RATE_SMOOTHING = 0.1;
} // namespace

bool MJPEG_HTTPCameraBackend::can_read_streams()
{
    return cv::videoio_registry::hasBackend(cv::CAP_FFMPEG);
}

const std::string MJPEG_HTTPCameraBackend::BACKEND_TYPE = "MJPEG_HTTP";

MJPEG_HTTPCameraBackend::MJPEG_HTTPCameraBackend(const Config &config)
    : CameraBackendBase(BACKEND_TYPE), config_(config)
{
    if (config_.url.empty())
    {
        throw std::logic_error("MJPEG_HTTPCameraBackend needs the URL of the stream");
    }
}

MJPEG_HTTPCameraBackend::~MJPEG_HTTPCameraBackend() { disconnect(); }

void MJPEG_HTTPCameraBackend::connect()
{
    if (running_)
    {
        return;
    }
    running_ = true;
    thread_ = std::thread(&MJPEG_HTTPCameraBackend::run_, this);
}

void MJPEG_HTTPCameraBackend::disconnect()
{
    running_ = false;
    if (thread_.joinable())
    {
        thread_.join();
    }
}

std::optional<CameraFrame> MJPEG_HTTPCameraBackend::latest_frame() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return frame_;
}

bool MJPEG_HTTPCameraBackend::is_alive() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return frame_ && seconds_since(frame_->received_at) <= config_.timeout;
}

bool MJPEG_HTTPCameraBackend::wait_for_frame(real_t timeout) const
{
    const auto start = clock_type::now();
    while (seconds_since(start) < timeout)
    {
        if (latest_frame())
        {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return latest_frame().has_value();
}

real_t MJPEG_HTTPCameraBackend::frame_rate() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return mean_interval_ > 0.0 ? 1.0 / mean_interval_ : 0.0;
}

void MJPEG_HTTPCameraBackend::run_()
{
    const int io_timeout_ms = static_cast<int>(1000.0 * config_.io_timeout);
    const std::vector<int> parameters{cv::CAP_PROP_OPEN_TIMEOUT_MSEC, io_timeout_ms,
                                      cv::CAP_PROP_READ_TIMEOUT_MSEC, io_timeout_ms};
    bool first_open = true;

    while (running_)
    {
        cv::VideoCapture capture;
        if (!first_open)
        {
            ++n_reconnects_;
        }
        first_open = false;

        if (capture.open(config_.url, cv::CAP_FFMPEG, parameters))
        {
            cv::Mat image;
            while (running_ && capture.read(image))
            {
                if (image.empty())
                {
                    continue;
                }

                const auto now = clock_type::now();
                std::lock_guard<std::mutex> lock(mutex_);
                if (frame_)
                {
                    const real_t interval =
                        std::chrono::duration<real_t>(now - frame_->received_at).count();
                    mean_interval_ =
                        mean_interval_ > 0.0
                            ? (1.0 - RATE_SMOOTHING) * mean_interval_ + RATE_SMOOTHING * interval
                            : interval;
                }
                const uint_t index = frame_ ? frame_->index + 1 : 1;
                // VideoCapture reuses its buffer, so keep a copy
                frame_ = CameraFrame{image.clone(), index, now};
            }
        }

        // The stream broke or could not be opened. Wait a little, but notice
        // a disconnect quickly
        const auto failed_at = clock_type::now();
        while (running_ && seconds_since(failed_at) < config_.reconnect_delay)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}

} // namespace sensors::backends
} // namespace bitrl

#endif

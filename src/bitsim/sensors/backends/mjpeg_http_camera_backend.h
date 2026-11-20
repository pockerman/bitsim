#ifndef MJPEG_HTTP_CAMERA_BACKEND_H
#define MJPEG_HTTP_CAMERA_BACKEND_H

#include "bitsim/bitsim_config.h"

#ifdef BITSIM_OPENCV

#include "bitsim/bitsim_types.h"
#include "bitsim/sensors/backends/camera_backend_base.h"

#include <atomic>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace bitsim
{
namespace sensors::backends
{

///
/// @brief Receives the frames of an MJPEG stream over HTTP, the
/// multipart/x-mixed-replace response that many cameras and camera servers
/// give, for example example 31's FastAPI server on a Raspberry Pi.
///
/// @details A thread of its own reads the stream and keeps only the latest
/// frame, so a slow reader never falls behind the camera. If the stream breaks
/// it reconnects. It uses OpenCV's FFmpeg backend to read the stream.
///
class MJPEG_HTTPCameraBackend final : public CameraBackendBase
{
  public:
    ///
    /// @brief The backend type
    ///
    static const std::string BACKEND_TYPE;

    ///
    /// @brief Settings of the backend
    ///
    struct Config
    {
        ///
        /// @brief The URL of the stream, e.g. http://raspberrypi.local:8000/stream
        ///
        std::string url;

        ///
        /// @brief The stream is alive while frames arrive at most this far
        /// apart [s]
        ///
        real_t timeout{1.0};

        ///
        /// @brief How long to wait before connecting again after a failure [s]
        ///
        real_t reconnect_delay{0.5};

        ///
        /// @brief Give up opening, or reading a frame, after this long [s]
        ///
        real_t io_timeout{5.0};
    };

    explicit MJPEG_HTTPCameraBackend(const Config &config);
    ~MJPEG_HTTPCameraBackend() override;

    MJPEG_HTTPCameraBackend(const MJPEG_HTTPCameraBackend &) = delete;
    MJPEG_HTTPCameraBackend &operator=(const MJPEG_HTTPCameraBackend &) = delete;

    void connect() override;
    void disconnect() override;
    std::optional<CameraFrame> latest_frame() const override;
    bool is_alive() const override;

    ///
    /// @brief Wait until the first frame arrives
    /// @param timeout How long to wait at most [s]
    /// @return Whether a frame arrived
    ///
    bool wait_for_frame(real_t timeout) const;

    ///
    /// @brief The frame rate of the stream, averaged over the last frames
    /// [frames/s]. Zero before two frames have arrived
    ///
    real_t frame_rate() const;

    ///
    /// @brief Whether this OpenCV can read HTTP streams at all. The backend
    /// reads them with OpenCV's FFmpeg backend, which an OpenCV built without
    /// FFmpeg does not have; the backend then never receives a frame
    ///
    static bool can_read_streams();

    ///
    /// @brief How many times the stream had to be opened again
    ///
    uint_t n_reconnects() const noexcept { return n_reconnects_; }

    const Config &config() const noexcept { return config_; }

  private:
    void run_();

    Config config_;
    std::thread thread_;
    std::atomic<bool> running_{false};
    std::atomic<uint_t> n_reconnects_{0};

    mutable std::mutex mutex_;
    std::optional<CameraFrame> frame_;
    real_t mean_interval_{0.0};
};

} // namespace sensors::backends
} // namespace bitsim

#endif
#endif // MJPEG_HTTP_CAMERA_BACKEND_H

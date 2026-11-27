#ifndef BITSIM_SENSORS_BACKENDS_IMU_BACKEND_BASE_H
#define BITSIM_SENSORS_BACKENDS_IMU_BACKEND_BASE_H

#include "bitsim/bitsim_config.h"
#include "bitsim/bitsim_types.h"
#include "bitsim/extern/nlohmann/json/json.hpp"
#include "bitsim/sensors/backends/sensor_backend_base.h"

#include <array>
#include <memory>
#include <string>

#ifdef BITSIM_CHRONO
#include <any>
#include <unordered_map>
#endif

namespace bitsim
{
namespace sensors::backends
{

/**
 * @class IMUBackendBase
 * @ingroup bitsim_sensors_backends
 * @brief Base class for inertial measurement units with a three axis
 * accelerometer and a three axis gyroscope, such as an MPU-6050.
 *
 * A backend reads six values, along the axes of the body the IMU is on:
 *
 * - 0, 1, 2: the specific force [m/s^2], what the accelerometer measures. It
 *   is the acceleration minus gravity, so an IMU at rest reads 9.81 up
 * - 3, 4, 5: the angular velocity [rad/s], what the gyroscope measures
 */
class IMUBackendBase : public SensorBackendBase
{
  public:
    using Position = std::array<real_t, 3>;

    ///
    /// @brief Create a backend by the name a robot file gives it
    /// @param backend_type SIMULATION (or CHRONO) for an IMU simulated with
    /// Chrono, MQTT for a real one on a board
    /// @throws std::runtime_error If the type is unknown, or bitsim was built
    /// without what the backend needs
    ///
    static std::shared_ptr<IMUBackendBase> create(const std::string &backend_type);

    ///
    /// @return Where the IMU is on the body [m]. Its axes are the body's
    ///
    const Position &position() const noexcept { return position_; }
    void set_position(const Position &position) noexcept { position_ = position; }

    ///
    /// @return M/S^2 and RAD/S: the units of the accelerometer and the gyroscope
    ///
    std::string sensor_units() const override { return "M/S^2,RAD/S"; }

    ///
    /// @brief Read the settings of the IMU from its section of a robot or
    /// sensor file, all optional:
    ///
    /// - `frame_pos`: [x, y, z], where it is on the body [m]
    /// - `update_rate`, readings per second [1/s]
    ///
    /// Backends read their own keys as well
    ///
    virtual void load(const nlohmann::json &j);

    ///
    /// @brief Read the settings from a JSON file, see load()
    ///
    void load_from_json(const std::string &filename) override;

#ifdef BITSIM_CHRONO
    ///
    /// @brief Draw the IMU as a small box on the body. The properties are
    /// x_length, y_length, z_length (float_t) and chrono::ChColor
    ///
    void
    add_visual_shape(const std::unordered_map<std::string, std::any> &shape_properties) override;
#endif

  protected:
    explicit IMUBackendBase(const std::string &backend_type);

  private:
    Position position_{0.0, 0.0, 0.0};
};

inline IMUBackendBase::IMUBackendBase(const std::string &backend_type)
    : SensorBackendBase(backend_type)
{
    set_n_read_values(6);
}

} // namespace sensors::backends
} // namespace bitsim

#endif // BITSIM_SENSORS_BACKENDS_IMU_BACKEND_BASE_H

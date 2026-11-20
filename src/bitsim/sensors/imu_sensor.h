#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include "bitsim/bitsim_consts.h"
#include "bitsim/bitsim_types.h"
#include "bitsim/sensors/backends/imu_backend_base.h"
#include "bitsim/sensors/sensor_base.h"

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace bitsim
{
namespace sensors
{

/**
 * @class IMUSensor
 * @ingroup bitsim_sensors
 * @brief An inertial measurement unit: a three axis accelerometer and a three
 * axis gyroscope, along the axes of the body it is on.
 *
 * read_values() gives six values: the specific force [m/s^2], then the angular
 * velocity [rad/s]; see backends::IMUBackendBase. acceleration() and
 * angular_velocity() pick them out of the last read.
 * backends::CHRONO_IMUBackend simulates the IMU, and
 * backends::MQTT_IMUBackend reads a real one on a board.
 */
class IMUSensor final : public SensorBase
{
  public:
    static const std::string SENSOR_TYPE;

    ///
    /// @brief A sensor with a new backend of the given type, see
    /// backends::IMUBackendBase::create
    ///
    static std::shared_ptr<IMUSensor> create(const std::string &backend_type,
                                             const std::string &name);

    explicit IMUSensor(std::shared_ptr<backends::IMUBackendBase> backend,
                       const std::string &name = consts::INVALID_STR);

    /**
     * @brief Enable the sensor, reading zeros until the first read
     */
    void init() override;

    /**
     * @brief Read the backend, if the sensor is enabled
     */
    const std::vector<real_t> &read_values() override;

    std::string backend_type_str() const override { return backend_->backend_type_str(); }

    std::string sensor_units() const override { return backend_->sensor_units(); }

    std::shared_ptr<backends::SensorBackendBase> backend_ptr() const override { return backend_; }

    std::shared_ptr<backends::IMUBackendBase> imu_backend() const noexcept { return backend_; }

    ///
    /// @return The specific force at the last read [m/s^2]
    ///
    std::array<real_t, 3> acceleration() const;

    ///
    /// @return The angular velocity at the last read [rad/s]
    ///
    std::array<real_t, 3> angular_velocity() const;

  private:
    std::shared_ptr<backends::IMUBackendBase> backend_;
};

} // namespace sensors
} // namespace bitsim

#endif // IMU_SENSOR_H

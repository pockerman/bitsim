#ifndef BITSIM_SENSORS_SENSOR_BASE_H
#define BITSIM_SENSORS_SENSOR_BASE_H

#include "../bitsim_consts.h"
#include "bitsim/bitsim_consts.h"
#include "bitsim/bitsim_types.h"
#include "bitsim/sensors/backends/sensor_backend_base.h"

#include <string>
#include <vector>
namespace bitsim
{
namespace sensors
{
/**
 * @class SensorBase
 * @ingroup bitsim_sensors
 * @brief Class for modelling sensors. A sensor is a device and has:
 * - sampling period
 * - enabled/disabled
 * - Last value
 * - Sensor units
 * Implementations can utilize noise and resolution.
 *
 */
class SensorBase
{
  public:
    /**
     * @brief Destructor
     */
    virtual ~SensorBase() = default;

    /**
     * @brief Initialize the sensor. Set the is is_enabled_ flag to true.
     * and performs any other initializations required by the sensor
     */
    virtual void init() = 0;

    /**
     * @return Reads the sensor values and updates the values held internally
     */
    virtual const std::vector<real_t> &read_values() = 0;

    /**
     * @return An instance of std::string of the backend type. If the implementation
     * does not use a specific backend returns bitsim::consts::INVALID_STR
     */
    virtual std::string backend_type_str() const = 0;

    /**
     * @return An instance of std::string indicating the units the sensor is using.
     * If units have not been established this returns bitsim::consts::INVALID_STR
     */
    virtual std::string sensor_units() const = 0;

    /**
     * @brief Returns true if the sensor is enabled
     * @return
     */
    bool is_enabled() const noexcept { return is_enabled_; }

    /**
     * @brief Set the is_enabled_ flag to true
     */
    void enable() noexcept { is_enabled_ = true; }

    /**
     * @brief Disable the sensor
     */
    void disable() noexcept { is_enabled_ = false; }

    /**
     * @return Read reference to the last values read by the sensor
     */
    const std::vector<real_t> &last_read_values() const noexcept { return values_; }

    /**
     * @return An instance of std::string with the name of the sensor
     */
    std::string sensor_name() const noexcept { return sensor_name_; }

    /**
     * @brief Set the sensor name
     * @param sensor_name The sensor name to set
     */
    void set_sensor_name(const std::string &sensor_name) noexcept { sensor_name_ = sensor_name; }

    /**
     * @return An instance of std::string with the type of the sensor
     */
    std::string sensor_type() const noexcept { return sensor_type_; }

    virtual std::shared_ptr<backends::SensorBackendBase> backend_ptr() const
    {
        return backend_ptr_;
    }

  protected:
    /**
     * @param sensor_type The type of the sensor
     * @param sensor_name The name of the sensor
     */
    explicit SensorBase(const std::string &sensor_type,
                        const std::string &sensor_name = bitsim::consts::INVALID_STR);

    /**
     * @brief The values last read by the sensor
     */
    std::vector<real_t> values_;

    ///
    /// @brief Pointer to backend tha actually handles the sensor
    ///
    std::shared_ptr<backends::SensorBackendBase> backend_ptr_;

  private:
    /**
     * @brief The type of the sensor
     */
    const std::string sensor_type_;

    /**
     * @brief The name of the sensor
     */
    std::string sensor_name_{bitsim::consts::INVALID_STR};

    /**
     * @brief Flag indicating if the sensor is enabled
     */
    bool is_enabled_{false};
};

inline SensorBase::SensorBase(const std::string &sensor_type, const std::string &sensor_name)
    : values_(), sensor_type_(sensor_type), sensor_name_(sensor_name)
{
}

} // namespace sensors
} // namespace bitsim

#endif // BITSIM_SENSORS_SENSOR_BASE_H

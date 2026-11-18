#ifndef RANGE_SENSOR_BACKEND_BASE_H
#define RANGE_SENSOR_BACKEND_BASE_H

#include "bitrl/bitrl_config.h"
#include "bitrl/bitrl_types.h"
#include "bitrl/extern/nlohmann/json/json.hpp"
#include "bitrl/sensors/backends/sensor_backend_base.h"
#include <memory>
#include <string>

#ifdef BITRL_CHRONO
#include <any>
#include <unordered_map>
#endif

namespace bitrl
{
namespace sensors::backends
{
/**
 * @class RangeSensorBackendBase
 * @ingroup bitrl_sensors_backends
 * @brief Base class for modelling range sensors
 *
 * A range sensor measures the distance to the nearest obstacle in front of
 * it, between min_distance() and max_distance(). Closer than min_distance()
 * it reports min_distance(), and with nothing in range max_distance().
 */
class RangeSensorBackendBase : public SensorBackendBase
{
  public:
    ///
    /// @brief Create a backend by the name a robot file gives it
    /// @param backend_type SIMULATION (or its old name CHRONO) for a sensor
    /// simulated with Chrono, MQTT for a real sensor on a board
    /// @throws std::runtime_error If the type is unknown, or bitrl was built
    /// without what the backend needs
    ///
    static std::shared_ptr<RangeSensorBackendBase> create(const std::string &backend_type);

    /**
     * @return The maximum distance the range sensor can read
     */
    real_t max_distance() const noexcept { return max_distance_; }

    /**
     * @brief Set the maximum distance the range sensor can read
     * @param max_distance
     */
    void set_max_distance(real_t max_distance) noexcept { max_distance_ = max_distance; }

    /**
     * @return The minimum distance the range sensor can read
     */
    real_t min_distance() const noexcept { return min_distance_; }

    /**
     * @brief Set the minimum distance the range sensor can read
     * @param min_distance
     */
    void set_min_distance(real_t min_distance) noexcept { min_distance_ = min_distance; }

    /**
     * @return An instance of std::string representing the units the sensor
     * readings are assumed in
     */
    virtual std::string sensor_units() const { return sensor_units_; }

    /**
     * @brief Set the sensor units
     * @param units The string representing the sensor units
     */
    void set_sensor_units(const std::string &units) { sensor_units_ = units; }

    ///
    /// @brief Read the settings of the sensor from its section of a robot or
    /// sensor file. Every key is optional:
    ///
    /// - `max_distance`, `min_distance` [m]
    /// - `update_rate`, readings per second [1/s]; the sampling period is its
    ///   inverse
    ///
    /// Backends read their own keys as well
    ///
    virtual void load(const nlohmann::json &j);

    ///
    /// @brief Read the settings from a JSON file, see load()
    ///
    void load_from_json(const std::string &filename) override;

#ifdef BITRL_CHRONO
    ///
    /// @brief Add a visual shape for the sensor in order to
    /// be visualized with Irrlicht. Concrete classes may choose not
    /// to implement this. In this case the sensor will not be visible
    ///
    virtual void
    add_visual_shape(const std::unordered_map<std::string, std::any> &shape_properties) override;
#endif

  protected:
    /**
     * @brief Constructor
     * @param backend_type
     */
    explicit RangeSensorBackendBase(const std::string &backend_type);

    ///
    /// @return The distance clamped to [min_distance(), max_distance()]
    ///
    real_t clamp_distance(real_t distance) const noexcept;

  private:
    std::string sensor_units_{"METERS"};

    /**
     *@brief The maximum distance the sensor can read
     */
    real_t max_distance_{0.0};

    /**
     *@brief The minimum distance the sensor can read
     */
    real_t min_distance_{0.0};
};

inline RangeSensorBackendBase::RangeSensorBackendBase(const std::string &backend_type)
    : SensorBackendBase(backend_type)
{
}

} // namespace sensors::backends
} // namespace bitrl

#endif // RANGE_SENSOR_BACKEND_BASE_H

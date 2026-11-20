#ifndef INFRARED_BACKEND_BASE_H
#define INFRARED_BACKEND_BASE_H

#include "bitsim/bitsim_config.h"
#include "bitsim/bitsim_types.h"
#include "bitsim/extern/nlohmann/json/json.hpp"
#include "bitsim/sensors/backends/sensor_backend_base.h"

#include <array>
#include <memory>
#include <string>
#include <vector>

#ifdef BITSIM_CHRONO
#include <any>
#include <unordered_map>
#endif

namespace bitsim
{
namespace sensors::backends
{

/**
 * @class InfraredBackendBase
 * @ingroup bitsim_sensors_backends
 * @brief Base class for arrays of infrared reflectance sensors, such as the
 * TCRT5000 modules that line following robots use.
 *
 * Each element shines infrared light at the floor just below it and measures
 * how much comes back. White paper reflects most of it and a black line very
 * little, so the elements tell the colour of the floor under them. A backend
 * reads one value per element: the reflectance, from 0 for black to 1 for
 * white.
 */
class InfraredBackendBase : public SensorBackendBase
{
  public:
    ///
    /// @brief The position of an element on the robot's body [m]
    ///
    using Position = std::array<real_t, 3>;

    ///
    /// @brief Create a backend by the name a robot file gives it
    /// @param backend_type SIMULATION (or CHRONO) for elements simulated with
    /// Chrono, MQTT for real elements on a board
    /// @throws std::runtime_error If the type is unknown, or bitsim was built
    /// without what the backend needs
    ///
    static std::shared_ptr<InfraredBackendBase> create(const std::string &backend_type);

    ///
    /// @return The positions of the elements on the body, in the order of the
    /// values
    ///
    const std::vector<Position> &elements() const noexcept { return elements_; }

    ///
    /// @brief Set the positions of the elements on the body. One value is read
    /// per element
    ///
    void set_elements(const std::vector<Position> &elements);

    ///
    /// @return The number of elements
    ///
    uint_t n_elements() const noexcept { return elements_.size(); }

    ///
    /// @return REFLECTANCE: the values are shares of the light, in [0, 1]
    ///
    std::string sensor_units() const override { return "REFLECTANCE"; }

    ///
    /// @brief Read the settings of the sensor from its section of a robot or
    /// sensor file:
    ///
    /// - `elements`: [[x, y, z], ...], the positions of the elements on the
    ///   body [m]
    /// - `update_rate`, readings per second [1/s], optional
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
    /// @brief Draw each element as a small box on the body. The properties
    /// are x_length, y_length, z_length (float_t) and chrono::ChColor
    ///
    void
    add_visual_shape(const std::unordered_map<std::string, std::any> &shape_properties) override;
#endif

  protected:
    explicit InfraredBackendBase(const std::string &backend_type);

  private:
    std::vector<Position> elements_;
};

inline InfraredBackendBase::InfraredBackendBase(const std::string &backend_type)
    : SensorBackendBase(backend_type)
{
    set_elements({Position{0.0, 0.0, 0.0}});
}

} // namespace sensors::backends
} // namespace bitsim

#endif // INFRARED_BACKEND_BASE_H

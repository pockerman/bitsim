#ifndef INFRARED_SENSOR_H
#define INFRARED_SENSOR_H

#include "bitsim/bitsim_consts.h"
#include "bitsim/bitsim_types.h"
#include "bitsim/sensors/backends/infrared_backend_base.h"
#include "bitsim/sensors/sensor_base.h"

#include <array>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace bitsim
{
namespace sensors
{

/**
 * @class InfraredSensor
 * @ingroup bitsim_sensors
 * @brief An array of infrared reflectance elements that looks at the floor,
 * to tell its colour: light paper or a dark line.
 *
 * read_values() gives the reflectance under each element, from 0 for black to
 * 1 for white. An element whose reflectance is below threshold() is over
 * something dark; see is_dark(). Where the values come from is up to the
 * backend: backends::CHRONO_InfraredBackend simulates the elements, and
 * backends::MQTT_InfraredBackend reads real ones on a board.
 */
class InfraredSensor final : public SensorBase
{
  public:
    /**
     * @brief The type of the sensor
     */
    static const std::string SENSOR_TYPE;

    ///
    /// @brief A sensor with a new backend of the given type, see
    /// backends::InfraredBackendBase::create
    ///
    static std::shared_ptr<InfraredSensor> create(const std::string &backend_type,
                                                  const std::string &name);

    explicit InfraredSensor(std::shared_ptr<backends::InfraredBackendBase> backend,
                            const std::string &name = consts::INVALID_STR);

    /**
     * @brief Enable the sensor, with every element reading white until the
     * first read
     */
    void init() override;

    /**
     * @brief Read the backend, if the sensor is enabled
     * @return The reflectance under each element
     */
    const std::vector<real_t> &read_values() override;

    std::string backend_type_str() const override { return backend_->backend_type_str(); }

    std::string sensor_units() const override { return backend_->sensor_units(); }

    std::shared_ptr<backends::SensorBackendBase> backend_ptr() const override { return backend_; }

    ///
    /// @return The backend, with the positions of the elements
    ///
    std::shared_ptr<backends::InfraredBackendBase> infrared_backend() const noexcept
    {
        return backend_;
    }

    ///
    /// @return The number of elements
    ///
    uint_t n_elements() const noexcept { return backend_->n_elements(); }

    ///
    /// @return Where the line is under the elements, in the body's frame [m]: the mean of the
    /// positions of the elements that see it, x and y; nothing if none does
    ///
    std::optional<std::array<real_t, 2>> line_position() const;

    ///
    /// @return Which elements are over something dark, left to right: "#" for dark and "."
    /// for light, such as ".#." with the line under the middle element
    ///
    std::string pattern() const;

    ///
    /// @brief Reflectances below this are dark
    ///
    real_t threshold() const noexcept { return threshold_; }
    void set_threshold(real_t threshold);

    ///
    /// @return True if element i was over something dark at the last read
    ///
    bool is_dark(uint_t i) const;

  private:
    std::shared_ptr<backends::InfraredBackendBase> backend_;
    real_t threshold_{0.5};
};

} // namespace sensors
} // namespace bitsim

#endif // INFRARED_SENSOR_H

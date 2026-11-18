#ifndef CHRONO_ULTRASOUND_BACKEND_H
#define CHRONO_ULTRASOUND_BACKEND_H

#include "bitrl/bitrl_config.h"

#ifdef BITRL_CHRONO

#include "bitrl/sensors/backends/range_sensor_backend_base.h"

#include <chrono/physics/ChBody.h>
#include <chrono/physics/ChSystem.h>

#include <any>
#include <memory>
#include <string>
#include <unordered_map>

namespace bitrl
{
namespace sensors::backends
{

/**
 * @class CHRONO_UltrasonicBackend
 * @ingroup bitrl_sensors_backends
 * @brief A simulated ultrasonic sensor: it casts rays into the Chrono world.
 *
 * The sensor sits at position() on the body it is attached to and looks
 * along the body's x axis. A real ultrasonic sensor hears echoes from a cone,
 * about 15 degrees wide for an HC-SR04, so the backend casts n_rays() rays
 * fanned across beam_angle() in the body's x-y plane and reports the nearest
 * hit. With one ray, the default, it looks straight ahead only.
 *
 * Only bodies with collision enabled reflect the rays. The body the sensor
 * is attached to never does.
 */
class CHRONO_UltrasonicBackend final : public RangeSensorBackendBase
{
  public:
    /**
     * @brief The type of the backend
     */
    static const std::string BACKEND_TYPE;

    ///
    /// @brief Constructor. Attach a body with set_chrono_body() before
    /// reading; the system is the one the body is in
    ///
    CHRONO_UltrasonicBackend();

    ///
    /// @brief Constructor
    /// @param sys The system the rays are cast in
    /// @param body The body the sensor is attached to
    ///
    CHRONO_UltrasonicBackend(chrono::ChSystem &sys, std::shared_ptr<chrono::ChBody> body);

    /**
     * @brief Read the distance to the nearest obstacle
     * @return One value, clamped to [min_distance(), max_distance()]
     * @throws std::logic_error If no body is attached, or the body is in no
     * system
     */
    std::vector<real_t> read_values() override;

    ///
    /// @brief Read the settings, see RangeSensorBackendBase::load(). Also
    /// reads, all optional:
    ///
    /// - `mounted_position` or `frame_pos`: [x, y, z] on the body [m]
    /// - `beam_angle`: the width of the cone [deg]
    /// - `n_rays`: how many rays to cast across it
    ///
    void load(const nlohmann::json &j) override;

    /**
     * @brief Set the position of the sensor on its body
     * @param pos
     */
    void set_position(const chrono::ChVector3d &pos) { position_ = pos; }

    /**
     * @return A read reference of the position of the sensor on its body
     */
    const chrono::ChVector3d &position() const { return position_; }

    ///
    /// @brief Set the cone the sensor hears echoes from
    /// @param beam_angle The full width of the cone [rad]
    /// @param n_rays How many rays to cast across it, at least 1
    ///
    void set_beam(real_t beam_angle, uint_t n_rays);

    ///
    /// @return The full width of the cone [rad]
    ///
    real_t beam_angle() const noexcept { return beam_angle_; }

    ///
    /// @return How many rays are cast across the cone
    ///
    uint_t n_rays() const noexcept { return n_rays_; }

  private:
    ///
    /// @brief The distance along one ray to the first body that is not the
    /// sensor's own, or max_distance()
    ///
    real_t cast_(const chrono::ChCollisionSystem &collision_sys, const chrono::ChVector3d &start,
                 const chrono::ChVector3d &dir) const;

    /**
     * @brief The system the rays are cast in, if given explicitly
     */
    chrono::ChSystem *sys_ptr_{nullptr};

    /**
     * @brief The position of the sensor on its body
     */
    chrono::ChVector3d position_{0.0, 0.0, 0.0};

    real_t beam_angle_{0.0};
    uint_t n_rays_{1};
};
} // namespace sensors::backends
} // namespace bitrl

#endif
#endif // CHRONO_ULTRASOUND_BACKEND_H

#ifndef CHRONO_INFRARED_BACKEND_H
#define CHRONO_INFRARED_BACKEND_H

#include "bitsim/bitsim_config.h"

#ifdef BITSIM_CHRONO

#include "bitsim/sensors/backends/infrared_backend_base.h"

#include <chrono/physics/ChBody.h>

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace bitsim
{
namespace sensors::backends
{

/**
 * @class CHRONO_InfraredBackend
 * @ingroup bitsim_sensors_backends
 * @brief Simulated infrared reflectance elements on a Chrono body.
 *
 * The floor is a function that gives its reflectance at each point (x, y) of
 * the world, from 0 for black to 1 for white; for a floor that is an image,
 * see sensors::FloorImage::reflectance. Each element reads the floor straight
 * below it, where the body carries it now.
 */
class CHRONO_InfraredBackend final : public InfraredBackendBase
{
  public:
    ///
    /// @brief The reflectance of the floor at a point of the world, in [0, 1]
    ///
    using Floor = std::function<real_t(real_t x, real_t y)>;

    static const std::string BACKEND_TYPE;

    ///
    /// @brief Constructor. Attach a body with set_chrono_body() and a floor
    /// with set_floor() before reading
    ///
    CHRONO_InfraredBackend();

    CHRONO_InfraredBackend(std::shared_ptr<chrono::ChBody> body, Floor floor);

    ///
    /// @brief Set the floor the elements look at
    ///
    void set_floor(Floor floor) { floor_ = std::move(floor); }

    bool has_floor() const noexcept { return static_cast<bool>(floor_); }

    ///
    /// @brief Read the floor under each element
    /// @return One value per element, in [0, 1]
    /// @throws std::logic_error If there is no body or no floor
    ///
    std::vector<real_t> read_values() override;

  private:
    Floor floor_;
};

} // namespace sensors::backends
} // namespace bitsim

#endif
#endif // CHRONO_INFRARED_BACKEND_H

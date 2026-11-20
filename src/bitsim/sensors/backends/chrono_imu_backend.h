#ifndef CHRONO_IMU_BACKEND_H
#define CHRONO_IMU_BACKEND_H

#include "bitsim/bitsim_config.h"

#ifdef BITSIM_CHRONO

#include "bitsim/sensors/backends/imu_backend_base.h"

#include <chrono/physics/ChBody.h>

#include <array>
#include <memory>
#include <random>
#include <string>
#include <vector>

namespace bitsim
{
namespace sensors::backends
{

/**
 * @class CHRONO_IMUBackend
 * @ingroup bitsim_sensors_backends
 * @brief A simulated IMU on a Chrono body.
 *
 * The accelerometer reads the acceleration of the point where the IMU sits,
 * minus the gravity of the system, and the gyroscope the angular velocity of
 * the body, both along the body's axes.
 *
 * A real gyroscope has a bias, a small rate it reads when it is still, and
 * noise. Integrated over time, the bias makes a heading drift by bias x time.
 * The backend adds both, if they are set: a fixed gyro_bias and normal noise
 * of standard deviation gyro_noise and accel_noise on every reading. The noise
 * comes from a generator with a fixed seed, so a simulation repeats.
 */
class CHRONO_IMUBackend final : public IMUBackendBase
{
  public:
    static const std::string BACKEND_TYPE;

    ///
    /// @brief How the IMU errs
    ///
    struct Errors
    {
        std::array<real_t, 3> gyro_bias{0.0, 0.0, 0.0}; ///< [rad/s]
        real_t gyro_noise{0.0};                         ///< standard deviation [rad/s]
        real_t accel_noise{0.0};                        ///< standard deviation [m/s^2]
        uint_t seed{42};

        ///
        /// @brief Read the keys gyro_bias, gyro_noise, accel_noise and seed,
        /// all optional
        ///
        void read(const nlohmann::json &j);
    };

    ///
    /// @brief Constructor. Attach a body with set_chrono_body() before reading
    ///
    CHRONO_IMUBackend();

    explicit CHRONO_IMUBackend(std::shared_ptr<chrono::ChBody> body);

    const Errors &errors() const noexcept { return errors_; }
    void set_errors(const Errors &errors);

    ///
    /// @brief Read the settings, see IMUBackendBase::load(), and the errors,
    /// see Errors::read()
    ///
    void load(const nlohmann::json &j) override;

    ///
    /// @brief Read the accelerometer and the gyroscope
    /// @return The specific force [m/s^2] and the angular velocity [rad/s]
    /// @throws std::logic_error If there is no body, or the body is in no
    /// system
    ///
    std::vector<real_t> read_values() override;

  private:
    Errors errors_;
    std::mt19937 generator_{42};
};

} // namespace sensors::backends
} // namespace bitsim

#endif
#endif // CHRONO_IMU_BACKEND_H

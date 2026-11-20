#ifndef DC_MOTOR_BACKEND_BASE_H
#define DC_MOTOR_BACKEND_BASE_H

#include "bitsim/bitsim_types.h"

#include <string>

namespace bitsim
{
namespace actuators
{

///
/// @brief The state of the output shaft of a motor, as a simulator sees it
///
struct ShaftState
{
    real_t angle{0.0}; ///< [rad]
    real_t speed{0.0}; ///< [rad/s]
};

namespace backends
{

/**
 * @class DCMotorBackendBase
 * @ingroup bitsim_actuators
 * @brief Base class for the implementations of a DC motor. A backend takes
 * a target speed for the output shaft and does whatever it takes to reach it:
 * a model of the motor, or a real motor on a board.
 *
 * The two kinds couple to a physics engine differently. A model computes the
 * torque the motor produces, so the engine moves the shaft (OutputType::TORQUE).
 * A real motor has already moved, so the engine has to follow its angle
 * (OutputType::ANGLE).
 */
class DCMotorBackendBase
{
  public:
    enum class OutputType
    {
        TORQUE,
        ANGLE
    };

    virtual ~DCMotorBackendBase() = default;

    ///
    /// @return How the backend couples to a physics engine
    ///
    virtual OutputType output_type() const noexcept = 0;

    ///
    /// @brief Set the target speed of the output shaft [rad/s]
    ///
    virtual void set_target_speed(real_t speed) = 0;

    ///
    /// @brief Advance the backend by dt. A model moves its own shaft
    ///
    virtual void step(real_t dt) = 0;

    ///
    /// @brief Advance the backend by dt, with the shaft moved by a physics
    /// engine. Backends with OutputType::ANGLE ignore the shaft
    ///
    virtual void step(real_t dt, const ShaftState &shaft) = 0;

    ///
    /// @return The angle of the output shaft [rad]
    ///
    virtual real_t angle() const = 0;

    ///
    /// @return The speed of the output shaft [rad/s]
    ///
    virtual real_t speed() const = 0;

    ///
    /// @return The angle of the output shaft as its encoder measures it [rad]:
    /// in whole counts, as a board would know it. Odometry uses this. Backends
    /// without an encoder return angle()
    ///
    virtual real_t encoder_angle() const { return angle(); }

    ///
    /// @return The torque on the output shaft [N m]. Zero for backends that
    /// cannot know it
    ///
    virtual real_t torque() const { return 0.0; }

    ///
    /// @return False while the backend cannot be used, for example before a
    /// board has reported
    ///
    virtual bool is_ready() const { return true; }

    ///
    /// @return The type of the backend
    ///
    const std::string &backend_type_str() const noexcept { return backend_type_; }

  protected:
    explicit DCMotorBackendBase(const std::string &backend_type) : backend_type_(backend_type) {}

  private:
    const std::string backend_type_;
};

} // namespace backends
} // namespace actuators
} // namespace bitsim

#endif // DC_MOTOR_BACKEND_BASE_H

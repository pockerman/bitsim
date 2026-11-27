#ifndef BITSIM_SYSTEMS_DOUBLE_PENDULUM_SYSTEM_H
#define BITSIM_SYSTEMS_DOUBLE_PENDULUM_SYSTEM_H

#include "bitsim/bitsim_config.h"

#ifdef BITSIM_CHRONO

#include "bitsim/bitsim_types.h"

#include <chrono/physics/ChBody.h>
#include <chrono/physics/ChSystemNSC.h>

#include <array>
#include <memory>

namespace bitsim
{
namespace systems
{

///
/// @brief The two links of a double pendulum, each a uniform rod
///
struct DoublePendulumProperties
{
    real_t l1 = 1.0;     ///< the length of the upper link [m]
    real_t l2 = 1.0;     ///< the length of the lower link [m]
    real_t m1 = 1.0;     ///< the mass of the upper link [kg]
    real_t m2 = 1.0;     ///< the mass of the lower link [kg]
    real_t width = 0.04; ///< the width of the links, as they are drawn [m]
};

/**
 * @class DoublePendulumSystem
 * @brief A double pendulum in Chrono: two uniform rods, the upper one hanging from a pivot at the
 * origin and the lower one from the end of the upper one, swinging in the x-z plane under
 * gravity, -z.
 *
 * The angles are those of the links from the downward vertical, positive towards +x:
 * \f$\theta = 0\f$ hangs straight down. There is no friction in the joints.
 */
class DoublePendulumSystem
{
  public:
    ///
    /// @param props The links
    /// @param theta1 The angle of the upper link to start from [rad]
    /// @param theta2 The angle of the lower link to start from [rad]
    ///
    explicit DoublePendulumSystem(const DoublePendulumProperties &props = {}, real_t theta1 = 0.0,
                                  real_t theta2 = 0.0);

    ///
    /// @brief Advance the simulation by dt [s]
    ///
    void step(real_t dt);

    ///
    /// @return The angles of the two links from the downward vertical [rad]
    ///
    std::array<real_t, 2> angles() const;

    ///
    /// @return The rates of change of the angles [rad/s]
    ///
    std::array<real_t, 2> angular_velocities() const;

    ///
    /// @return The kinetic plus the potential energy, with the pivot at zero height [J]
    ///
    real_t energy() const;

    real_t time() const { return system_.GetChTime(); }
    const DoublePendulumProperties &properties() const noexcept { return props_; }

    ///
    /// @brief The Chrono system, to draw it
    ///
    chrono::ChSystemNSC &system() noexcept { return system_; }

  private:
    DoublePendulumProperties props_;
    chrono::ChSystemNSC system_;
    std::shared_ptr<chrono::ChBody> link1_;
    std::shared_ptr<chrono::ChBody> link2_;
};

} // namespace systems
} // namespace bitsim

#endif

#endif // BITSIM_SYSTEMS_DOUBLE_PENDULUM_SYSTEM_H

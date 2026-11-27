#ifndef BITSIM_DYNAMICS_CART_POLE_DYNAMICS_H
#define BITSIM_DYNAMICS_CART_POLE_DYNAMICS_H

#include "bitsim/bitsim_types.h"

#include <Eigen/Dense>

namespace bitsim
{
namespace dynamics
{

///
/// @brief The cart and the pole. The defaults are those of the classic cart-pole of Barto,
/// Sutton and Anderson (1983), which Gymnasium's CartPole uses too
///
struct CartPoleParameters
{
    real_t cart_mass{1.0};         ///< [kg]
    real_t pole_mass{0.1};         ///< [kg]
    real_t pole_half_length{0.5};  ///< from the pivot to the centre of the pole [m]
    real_t gravity{9.8};           ///< [m/s^2]
    real_t max_force{10.0};        ///< the largest push on the cart, either way [N]
    real_t track_half_length{2.4}; ///< the cart must stay within this of the centre [m]
};

/// @brief The state: the position of the cart [m], its velocity [m/s], the angle of the pole
/// from upright [rad], positive when it leans towards +x, and its rate [rad/s]
using CartPoleState = RealColVec4d;

/// @brief Where each value is in a CartPoleState
namespace cart_pole
{
constexpr int X = 0;
constexpr int X_DOT = 1;
constexpr int THETA = 2;
constexpr int THETA_DOT = 3;
} // namespace cart_pole

///
/// @brief A model linearised about a state and a force: \f$\dot x \approx A x + B F\f$, for the
/// deviations from them
///
struct CartPoleLinearModel
{
    RealMat4d A;
    RealColVec4d B;
};

/**
 * @class CartPoleDynamics
 * @brief A cart on a straight track, pushed by a horizontal force F, with a pole hinged on it.
 *
 * The pole is a uniform rod of mass m with its centre l from the pivot, on a cart of mass M,
 * without friction. With \f$\theta\f$ the angle of the pole from upright,
 * \f[
 * \ddot\theta = \frac{g \sin\theta - \cos\theta \, \tau}{l \left(\tfrac{4}{3} -
 * \tfrac{m \cos^2\theta}{M + m}\right)}, \quad
 * \ddot x = \tau - \frac{m l \ddot\theta \cos\theta}{M + m}, \quad
 * \tau = \frac{F + m l \dot\theta^2 \sin\theta}{M + m} ,
 * \f]
 * the equations of Barto, Sutton and Anderson. Upright, \f$\theta = 0\f$, is an equilibrium, and
 * an unstable one: the model is the classic test of a controller that has to balance something.
 *
 * step() integrates with fourth order Runge-Kutta, with the force first limited to max_force.
 * The track is not a wall: off_track() says when the cart has gone past its end, and the
 * application decides what that means.
 */
class CartPoleDynamics
{
  public:
    explicit CartPoleDynamics(const CartPoleParameters &parameters = CartPoleParameters(),
                              const CartPoleState &state = CartPoleState::Zero());

    ///
    /// @brief Advance by dt [s], with the force F [N] held over the step
    ///
    void step(real_t force, real_t dt);

    ///
    /// @return The derivative of the state, for the state and the force, which is not limited
    ///
    CartPoleState derivative(const CartPoleState &x, real_t force) const;

    ///
    /// @return The model linearised about the state x and the force, by central differences
    ///
    CartPoleLinearModel linearise(const CartPoleState &x = CartPoleState::Zero(),
                                  real_t force = 0.0) const;

    ///
    /// @return The force limited to [-max_force, max_force]
    ///
    real_t limit(real_t force) const;

    ///
    /// @return The kinetic and the potential energy, the potential from the pivot
    ///
    real_t energy(const CartPoleState &x) const;

    ///
    /// @return Whether the cart is past either end of the track
    ///
    bool off_track() const;

    const CartPoleState &state() const noexcept { return state_; }
    CartPoleState &state() noexcept { return state_; }
    const CartPoleParameters &parameters() const noexcept { return p_; }
    real_t time() const noexcept { return time_; }

  private:
    CartPoleParameters p_;
    CartPoleState state_;
    real_t time_{0.0};
};

} // namespace dynamics
} // namespace bitsim

#endif // BITSIM_DYNAMICS_CART_POLE_DYNAMICS_H

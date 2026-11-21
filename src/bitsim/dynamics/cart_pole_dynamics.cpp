#include "bitsim/dynamics/cart_pole_dynamics.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace bitsim
{
namespace dynamics
{

CartPoleDynamics::CartPoleDynamics(const CartPoleParameters &parameters, const CartPoleState &state)
    : p_(parameters), state_(state)
{
    if (!(p_.cart_mass > 0.0 && p_.pole_mass > 0.0 && p_.pole_half_length > 0.0 &&
          p_.max_force >= 0.0 && p_.track_half_length > 0.0))
    {
        throw std::invalid_argument("CartPoleDynamics: the masses, the length of the pole and the "
                                    "track must be positive");
    }
}

CartPoleState CartPoleDynamics::derivative(const CartPoleState &x, real_t force) const
{
    using namespace cart_pole;
    const real_t M = p_.cart_mass;
    const real_t m = p_.pole_mass;
    const real_t l = p_.pole_half_length;
    const real_t total = M + m;
    const real_t s = std::sin(x[THETA]);
    const real_t c = std::cos(x[THETA]);
    const real_t w = x[THETA_DOT];

    const real_t tau = (force + m * l * w * w * s) / total;
    const real_t theta_acc = (p_.gravity * s - c * tau) / (l * (4.0 / 3.0 - m * c * c / total));
    const real_t x_acc = tau - m * l * theta_acc * c / total;

    CartPoleState d;
    d << x[X_DOT], x_acc, w, theta_acc;
    return d;
}

real_t CartPoleDynamics::limit(real_t force) const
{
    return std::clamp(force, -p_.max_force, p_.max_force);
}

void CartPoleDynamics::step(real_t force, real_t dt)
{
    const real_t F = limit(force);
    const CartPoleState k1 = derivative(state_, F);
    const CartPoleState k2 = derivative(state_ + 0.5 * dt * k1, F);
    const CartPoleState k3 = derivative(state_ + 0.5 * dt * k2, F);
    const CartPoleState k4 = derivative(state_ + dt * k3, F);
    state_ += dt / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
    time_ += dt;
}

CartPoleLinearModel CartPoleDynamics::linearise(const CartPoleState &x, real_t force) const
{
    constexpr real_t h = 1.0e-6;
    CartPoleLinearModel model;
    for (int j = 0; j < 4; ++j)
    {
        CartPoleState plus = x;
        CartPoleState minus = x;
        plus[j] += h;
        minus[j] -= h;
        model.A.col(j) = (derivative(plus, force) - derivative(minus, force)) / (2.0 * h);
    }
    model.B = (derivative(x, force + h) - derivative(x, force - h)) / (2.0 * h);
    return model;
}

real_t CartPoleDynamics::energy(const CartPoleState &x) const
{
    using namespace cart_pole;
    const real_t M = p_.cart_mass;
    const real_t m = p_.pole_mass;
    const real_t l = p_.pole_half_length;
    const real_t v = x[X_DOT];
    const real_t w = x[THETA_DOT];
    const real_t c = std::cos(x[THETA]);

    // the centre of the pole moves at (v + l w cos, -l w sin); its inertia about it is m l^2 / 3
    const real_t kinetic = 0.5 * M * v * v +
                           0.5 * m * (v * v + 2.0 * v * l * w * c + l * l * w * w) +
                           0.5 * (m * l * l / 3.0) * w * w;
    const real_t potential = m * p_.gravity * l * c;
    return kinetic + potential;
}

bool CartPoleDynamics::off_track() const
{
    return std::abs(state_[cart_pole::X]) > p_.track_half_length;
}

} // namespace dynamics
} // namespace bitsim

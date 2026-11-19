#include "bitrl/dynamics/bicycle_vehicle_model_dynamics.h"
#include "bitrl/utils/std_map_utils.h"

#include <array>
#include <cmath>

namespace bitrl
{
namespace dynamics
{

BicycleVehicleModelDynamics::BicycleVehicleModelDynamics(
    const BicycleVehicleModelDynamicsConfig config, SysState<3> &init_state)
    : BicycleVehicleModelDynamics::base_type(init_state), config_(config)
{
}

void BicycleVehicleModelDynamics::integrate(const BicycleVehicleModelDynamics::input_type &input)
{

    auto old_x = this->state_.get("X");
    auto old_y = this->state_.get("Y");
    auto old_theta = this->state_.get("Theta");
    auto dt = config_.dt;

    auto v = bitrl::utils::template resolve<real_t>("v", input);
    auto delta = bitrl::utils::template resolve<real_t>("delta", input);

    // the input error for v and delta
    auto errors = bitrl::utils::template resolve<std::array<real_t, 2>>("errors", input);

    // The Jacobians are those of this step, at the state before it
    if (this->allows_matrix_updates())
    {
        update_matrices(input);
    }

    auto x_new = old_x + (v + errors[0]) * std::cos(old_theta) * dt;
    auto y_new = old_y + (v + errors[0]) * std::sin(old_theta) * dt;
    auto theta_new = old_theta + (v + errors[0]) * std::tan(delta + errors[1]) * dt / config_.L;

    this->state_[0] = x_new;
    this->state_[1] = y_new;
    this->state_[2] = theta_new; // we may want to clip this value
}

BicycleVehicleModelDynamics::state_type &
BicycleVehicleModelDynamics::evaluate(const BicycleVehicleModelDynamics::input_type &input)
{
    integrate(input);
    return this->get_state();
}

void BicycleVehicleModelDynamics::initialize_matrices(
    const BicycleVehicleModelDynamics::input_type &input)
{
    set_matrix_update_flag(true);
    update_matrices(input);
}

void BicycleVehicleModelDynamics::update_matrices(
    const BicycleVehicleModelDynamics::input_type &input)
{
    // F = d(x', y', theta') / d(x, y, theta) and L = d(x', y', theta') / d(e0, e1), the errors in
    // the speed and in the steering angle
    if (!this->has_matrix("F"))
    {
        this->set_matrix("F", matrix_type::Identity(3, 3));
    }
    if (!this->has_matrix("L"))
    {
        this->set_matrix("L", matrix_type::Zero(3, 2));
    }

    const auto v = bitrl::utils::template resolve<real_t>("v", input);
    const auto delta = bitrl::utils::template resolve<real_t>("delta", input);
    const auto errors = bitrl::utils::template resolve<std::array<real_t, 2>>("errors", input);
    const real_t dt = config_.dt;
    const real_t speed = v + errors[0];
    const real_t steer = delta + errors[1];
    const real_t theta = this->state_.get("Theta");
    const real_t c = std::cos(theta);
    const real_t s = std::sin(theta);

    auto &F = this->get_matrix("F");
    F.setIdentity();
    F(0, 2) = -speed * s * dt;
    F(1, 2) = speed * c * dt;

    auto &L = this->get_matrix("L");
    L.setZero();
    L(0, 0) = c * dt;
    L(1, 0) = s * dt;
    L(2, 0) = std::tan(steer) * dt / config_.L;
    L(2, 1) = speed * dt / (config_.L * std::cos(steer) * std::cos(steer));
}

} // namespace dynamics
} // namespace bitrl

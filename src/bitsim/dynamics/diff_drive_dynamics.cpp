#include "bitsim/dynamics/diff_drive_dynamics.h"
#include "bitsim/bitsim_consts.h"
#include "bitsim/maths/math_utils.h"
#include "bitsim/utils/std_map_utils.h"

#include <cmath>

namespace bitsim
{
namespace dynamics
{

namespace
{
// One step of V1: the robot drives an arc of length d and turns by phi. With small phi the arc
// formulas divide by almost zero, and the robot moves along the heading halfway through the turn
// instead, which is what the arc tends to
struct ArcStep
{
    real_t d;   // the distance driven, v dt + e0
    real_t phi; // the turn, w dt + e1
    bool straight;
};

ArcStep arc_step(real_t v, real_t w, real_t dt, real_t tol, const std::array<real_t, 2> &errors)
{
    const real_t d = v * dt + errors[0];
    const real_t phi = w * dt + errors[1];
    return {d, phi, std::fabs(phi) < tol};
}
} // namespace

SysState<3> DiffDriveDynamics::integrate_state_v1(const SysState<3> &state, real_t tol, real_t dt,
                                                  real_t v, real_t w,
                                                  const std::array<real_t, 2> &errors)
{
    const ArcStep step = arc_step(v, w, dt, tol, errors);
    const real_t theta = state[2];
    const real_t theta_new = theta + step.phi;

    SysState<3> other(state);
    if (step.straight)
    {
        other[0] += step.d * std::cos(theta + 0.5 * step.phi);
        other[1] += step.d * std::sin(theta + 0.5 * step.phi);
    }
    else
    {
        const real_t radius = step.d / step.phi;
        other[0] += radius * (std::sin(theta_new) - std::sin(theta));
        other[1] -= radius * (std::cos(theta_new) - std::cos(theta));
    }
    other[2] = bitsim::maths::wrap_angle(theta_new);
    return other;
}

SysState<3> DiffDriveDynamics::integrate_state_v2(const SysState<3> &state, real_t dt, real_t v,
                                                  real_t w, const std::array<real_t, 2> &errors)
{

    auto values = state.get_values();
    auto distance = v * dt; // 0.5*v*dt;
    auto xincrement = (distance + errors[0]) * std::cos(values[2] + errors[1]);
    auto yincrement = (distance + errors[0]) * std::sin(values[2] + errors[1]);

    SysState<3> other(state);

    other[0] += xincrement;
    other[1] += yincrement;
    other[2] += dt * w;
    return other;
}

SysState<3> DiffDriveDynamics::integrate_state_v3(const SysState<3> &state, real_t r, real_t l,
                                                  real_t dt, real_t w1, real_t w2,
                                                  const std::array<real_t, 2> &errors)
{

    auto values = state.get_values();
    auto xincrement = dt * 0.5 * r * (w1 + w2 + errors[0]) * std::cos(values[2] + errors[1]);
    auto yincrement = dt * 0.5 * r * (w1 + w2 + errors[0]) * std::sin(values[2] + errors[1]);

    SysState<3> other(state);
    other[0] += xincrement;
    other[1] += yincrement;
    other[2] += dt * r * (w1 - w2) / (2.0 * l);
    return other;
}

SysState<3> DiffDriveDynamics::integrate(const SysState<3> &state,
                                         const DiffDriveDynamics::input_type &input,
                                         const DynamicVersion version)
{

    auto result = state;
    if (version == DiffDriveDynamics::DynamicVersion::V1)
    {

        auto w = bitsim::utils::template resolve<real_t>("w", input);
        auto v = bitsim::utils::template resolve<real_t>("v", input);
        auto errors = bitsim::utils::template resolve<std::array<real_t, 2>>("errors", input);
        auto dt = bitsim::utils::template resolve<real_t>("dt", input);
        auto tol = bitsim::utils::template resolve<real_t>("tol", input);
        result = DiffDriveDynamics::integrate_state_v1(state, tol, dt, v, w, errors);
    }
    else if (version == DiffDriveDynamics::DynamicVersion::V2)
    {

        auto w = bitsim::utils::template resolve<real_t>("w", input);
        auto v = bitsim::utils::template resolve<real_t>("v", input);
        auto errors = bitsim::utils::template resolve<std::array<real_t, 2>>("errors", input);
        auto dt = bitsim::utils::template resolve<real_t>("dt", input);
        result = DiffDriveDynamics::integrate_state_v2(state, dt, v, w, errors);
    }
    else if (version == DiffDriveDynamics::DynamicVersion::V3)
    {

        // in this scenario we have the wheels speed as input

        auto w1 = bitsim::utils::template resolve<real_t>("w1", input);
        auto w2 = bitsim::utils::template resolve<real_t>("w2", input);
        auto errors = bitsim::utils::template resolve<std::array<real_t, 2>>("errors", input);
        auto r = bitsim::utils::template resolve<real_t>("r", input);
        auto l = bitsim::utils::template resolve<real_t>("l", input);
        auto dt = bitsim::utils::template resolve<real_t>("dt", input);
        result = DiffDriveDynamics::integrate_state_v3(state, r, l, dt, w1, w2, errors);
    }

    return result;
}

DiffDriveDynamics::DiffDriveDynamics(DiffDriveDynamics::DynamicVersion type,
                                     bool update_description_matrices_on_evaluate)
    : MotionModelDynamicsBase<SysState<3>, DynamicsMatrixDescriptor,
                              std::map<std::string, std::any>>(
          update_description_matrices_on_evaluate),
      v_(0.0), w_(0.0), type_(type)
{
    this->state_.set(0, {"X", 0.0});
    this->state_.set(1, {"Y", 0.0});
    this->state_.set(2, {"Theta", 0.0});
}

DiffDriveDynamics::DiffDriveDynamics(DiffDriveDynamics::state_type &&state)
    : MotionModelDynamicsBase<SysState<3>, DynamicsMatrixDescriptor,
                              std::map<std::string, std::any>>(),
      v_(0.0), w_(0.0)
{
    this->state_ = state;
}

void DiffDriveDynamics::integrate(const DiffDriveDynamics::input_type &input)
{

    // before we do the integration
    // update the matrices
    if (this->allows_matrix_updates())
    {
        update_matrices(input);
    }

    if (type_ == DiffDriveDynamics::DynamicVersion::V1)
    {

        auto w = bitsim::utils::template resolve<real_t>("w", input);
        auto v = bitsim::utils::template resolve<real_t>("v", input);
        auto errors = bitsim::utils::template resolve<std::array<real_t, 2>>("errors", input);
        auto result = DiffDriveDynamics::integrate_state_v1(this->state_, tol_, get_time_step(), v,
                                                            w, errors);
        this->state_ = result;

        // update the velocities and angular velocities
        v_ = v;
        w_ = w;
    }
    else if (type_ == DiffDriveDynamics::DynamicVersion::V2)
    {

        auto w = bitsim::utils::template resolve<real_t>("w", input);
        auto v = bitsim::utils::template resolve<real_t>("v", input);
        auto errors = bitsim::utils::template resolve<std::array<real_t, 2>>("errors", input);
        auto result =
            DiffDriveDynamics::integrate_state_v2(this->state_, get_time_step(), v, w, errors);
        this->state_ = result;

        // update the velocities and angular
        // velocities
        v_ = v;
        w_ = w;
    }
    else if (type_ == DiffDriveDynamics::DynamicVersion::V3)
    {

        // in this scenario we have the wheels speed as input

        auto w1 = bitsim::utils::template resolve<real_t>("w1", input);
        auto w2 = bitsim::utils::template resolve<real_t>("w2", input);
        auto errors = bitsim::utils::template resolve<std::array<real_t, 2>>("errors", input);
        auto r = bitsim::utils::template resolve<real_t>("r", input);
        auto l = bitsim::utils::template resolve<real_t>("l", input);
        auto result = DiffDriveDynamics::integrate_state_v3(this->state_, r, l, get_time_step(), w1,
                                                            w2, errors);
        this->state_ = result;

        v_ = 0.5 * r * (w1 + w2);
        w_ = r * (w1 - w2) / (2.0 * l);
    }
}

DiffDriveDynamics::state_type &
DiffDriveDynamics::evaluate(const DiffDriveDynamics::input_type &input)
{
    integrate(input);
    return this->state_;
}

void DiffDriveDynamics::initialize_matrices(const DiffDriveDynamics::input_type &input)
{

    // if we initialize the matrices
    // then we should set the matrix update flag to true
    set_matrix_update_flag(true);

    if (!this->has_matrix("F"))
    {
        matrix_type F = matrix_type::Zero(3, 3); //(3,3, 0.0);
        this->set_matrix("F", F);
    }

    if (!this->has_matrix("L"))
    {
        matrix_type L = matrix_type::Zero(3, 2); //, 0.0);
        this->set_matrix("L", L);
    }

    update_matrices(input);
}

void DiffDriveDynamics::update_matrices(const DiffDriveDynamics::input_type &input)
{
    // The Jacobians of the V1 step at the current state: F = d(x', y', theta') / d(x, y, theta)
    // and L = d(x', y', theta') / d(e0, e1), the errors in the distance and in the turn
    auto w = bitsim::utils::template resolve<real_t>("w", input);
    auto v = bitsim::utils::template resolve<real_t>("v", input);
    auto errors = bitsim::utils::template resolve<std::array<real_t, 2>>("errors", input);

    const ArcStep step = arc_step(v, w, get_time_step(), tol_, errors);
    const real_t theta = this->state_[2];
    const real_t theta_new = theta + step.phi;

    auto &F = this->get_matrix("F");
    auto &L = this->get_matrix("L");
    F.setIdentity();
    L.setZero();
    L(2, 1) = 1.0;

    if (step.straight)
    {
        const real_t mid = theta + 0.5 * step.phi;
        F(0, 2) = -step.d * std::sin(mid);
        F(1, 2) = step.d * std::cos(mid);

        L(0, 0) = std::cos(mid);
        L(0, 1) = -0.5 * step.d * std::sin(mid);
        L(1, 0) = std::sin(mid);
        L(1, 1) = 0.5 * step.d * std::cos(mid);
    }
    else
    {
        const real_t ds = std::sin(theta_new) - std::sin(theta);
        const real_t dc = std::cos(theta_new) - std::cos(theta);
        const real_t radius = step.d / step.phi;
        F(0, 2) = radius * dc;
        F(1, 2) = radius * ds;

        L(0, 0) = ds / step.phi;
        L(0, 1) = radius * (std::cos(theta_new) - ds / step.phi);
        L(1, 0) = -dc / step.phi;
        L(1, 1) = radius * (std::sin(theta_new) + dc / step.phi);
    }
}

} // namespace dynamics
} // namespace bitsim

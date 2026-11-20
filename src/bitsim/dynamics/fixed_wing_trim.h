/**
 * @file fixed_wing_trim.h
 * @brief The trim of a fixed wing aircraft for level flight, its linear model about a trim, and
 * the modes of that model.
 */

#ifndef BITSIM_DYNAMICS_FIXED_WING_TRIM_H
#define BITSIM_DYNAMICS_FIXED_WING_TRIM_H

#include "bitsim/bitsim_types.h"
#include "bitsim/dynamics/fixed_wing_dynamics.h"

#include <Eigen/Dense>

#include <complex>
#include <string>
#include <vector>

namespace bitsim::dynamics
{

///
/// @brief A steady flight: the state and the controls that keep it
///
struct FixedWingTrim
{
    FixedWingState state{FixedWingState::Zero()};
    FixedWingControls controls{};
    real_t airspeed{0.0}; ///< [m/s]
    real_t alpha{0.0};    ///< [rad]
    real_t beta{0.0};     ///< [rad]
    real_t residual{0.0}; ///< the largest acceleration left [m/s^2 or rad/s^2]
    bool converged{false};
};

///
/// @brief Find the controls for straight and level flight at the given airspeed, with no wind.
/// The unknowns are the angle of attack and the sideslip, and the elevator, aileron, rudder and
/// throttle; the pitch equals the angle of attack and the roll is 0. The search ignores the
/// limits of the controls; converged says whether it found a trim, and the controls are within
/// them
/// @param parameters The aircraft
/// @param airspeed [m/s]
/// @param altitude [m], above the ground so that the wheels do not touch
/// @param heading [rad]
///
FixedWingTrim trim_level_flight(const FixedWingParameters &parameters, real_t airspeed,
                                real_t altitude = 100.0, real_t heading = 0.0);

///
/// @brief The motion near a trim, \f$\dot{\delta x} = A \delta x + B \delta u\f$, with the
/// controls in the order elevator, aileron, rudder, throttle
///
struct FixedWingLinearModel
{
    Eigen::Matrix<real_t, 12, 12, Eigen::DontAlign> A;
    Eigen::Matrix<real_t, 12, 4, Eigen::DontAlign> B;
};

///
/// @brief Linearise the dynamics about a trim, by central differences
///
FixedWingLinearModel linearise(const FixedWingParameters &parameters, const FixedWingTrim &trim);

///
/// @brief A mode of the linearised motion
///
struct FlightMode
{
    std::string name;
    std::complex<real_t> eigenvalue;
    real_t natural_frequency{0.0}; ///< [rad/s]
    real_t damping{0.0};           ///< the damping ratio; for a real mode 1 if it decays
    real_t period{0.0};            ///< of an oscillation [s]; 0 for a real mode
    real_t time_constant{0.0};     ///< 1 / |Re lambda| [s]; negative if the mode grows
};

///
/// @brief The classical modes: short period and phugoid from the longitudinal states
/// (u, w, q, theta), and roll, spiral and Dutch roll from the lateral ones (v, p, r, phi)
///
std::vector<FlightMode> flight_modes(const FixedWingLinearModel &model);

} // namespace bitsim::dynamics

#endif // BITSIM_DYNAMICS_FIXED_WING_TRIM_H

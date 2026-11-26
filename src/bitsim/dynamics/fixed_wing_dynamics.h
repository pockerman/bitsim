#ifndef BITSIM_DYNAMICS_FIXED_WING_DYNAMICS_H
#define BITSIM_DYNAMICS_FIXED_WING_DYNAMICS_H

#include "bitsim/bitsim_types.h"

#include <Eigen/Dense>

#include <array>
#include <vector>

namespace bitsim::dynamics
{

///
/// @brief The 12 values of the state of a fixed wing aircraft, in this order: the position
/// (north, east, down) in the world [m], the velocity (u, v, w) in the body frame [m/s], the Euler
/// angles (roll, pitch, yaw) [rad] and the angular velocity (p, q, r) in the body frame [rad/s]
///
using FixedWingState = UnalignedMat<real_t, 12, 1>;

///
/// @brief Where each value is in a FixedWingState
///
namespace fw
{
enum Index : int
{
    PN = 0,
    PE = 1,
    PD = 2,
    U = 3,
    V = 4,
    W = 5,
    PHI = 6,
    THETA = 7,
    PSI = 8,
    P = 9,
    Q = 10,
    R = 11
};
} // namespace fw

///
/// @brief What the pilot or the autopilot sets
///
struct FixedWingControls
{
    real_t elevator{0.0}; ///< [rad]; positive is trailing edge down, which pitches the nose down
    real_t aileron{0.0};  ///< [rad]; positive rolls the aircraft right
    real_t rudder{0.0};   ///< [rad]; positive yaws the nose left, and steers the nose wheel left
    real_t throttle{0.0}; ///< in [0, 1], up to FixedWingParameters::max_throttle
    real_t brake{0.0};    ///< of the main wheels, in [0, 1]
};

///
/// @brief A wheel of the landing gear: a spring and damper along the body z axis, that rolls
/// along its heading and grips sideways
///
struct LandingGearWheel
{
    std::array<real_t, 3> position; ///< where it touches the ground, in the body frame [m]
    real_t stiffness{2000.0};       ///< [N/m]
    real_t damping{120.0};          ///< [N s/m]
    bool steers{false};             ///< the nose wheel turns with the rudder
    bool brakes{false};             ///< the main wheels brake
};

///
/// @brief The aircraft: its mass, its geometry, its aerodynamic coefficients, its propeller and
/// motor, and its landing gear. aerosonde() gives the small UAV of Beard and McLain
///
struct FixedWingParameters
{
    // Mass and inertia
    real_t mass{11.0};  ///< [kg]
    real_t Jx{0.8244};  ///< [kg m^2]
    real_t Jy{1.135};   ///< [kg m^2]
    real_t Jz{1.759};   ///< [kg m^2]
    real_t Jxz{0.1204}; ///< [kg m^2]

    // Geometry and air
    real_t S{0.55};     ///< the wing area [m^2]
    real_t b{2.8956};   ///< the wing span [m]
    real_t c{0.18994};  ///< the mean chord [m]
    real_t rho{1.2682}; ///< the density of the air [kg/m^3]
    real_t e{0.9};      ///< the Oswald efficiency of the wing

    // Longitudinal coefficients
    real_t CL0{0.23}, CL_alpha{5.61}, CL_q{7.95}, CL_de{0.13};
    real_t CD0{0.043}, CD_q{0.0}, CD_de{0.0135};
    real_t Cm0{0.0135}, Cm_alpha{-2.74}, Cm_q{-38.21}, Cm_de{-0.99};
    real_t stall_alpha{0.47};     ///< the angle of attack at which the wing stalls [rad]
    real_t stall_sharpness{50.0}; ///< how suddenly it stalls

    // Lateral coefficients
    real_t CY0{0.0}, CY_beta{-0.98}, CY_p{0.0}, CY_r{0.0}, CY_da{0.075}, CY_dr{0.19};
    real_t Cl0{0.0}, Cl_beta{-0.13}, Cl_p{-0.51}, Cl_r{0.25}, Cl_da{0.17}, Cl_dr{0.0024};
    real_t Cn0{0.0}, Cn_beta{0.073}, Cn_p{-0.069}, Cn_r{-0.095}, Cn_da{-0.011}, Cn_dr{-0.069};

    // Propeller and motor
    real_t prop_diameter{0.508};                        ///< [m]
    real_t motor_KV{145.0};                             ///< [rpm/V]
    real_t motor_R{0.042};                              ///< [ohm]
    real_t motor_i0{1.5};                               ///< the no-load current [A]
    real_t battery_V{44.4};                             ///< the voltage at full throttle [V]
    real_t CT0{0.09357}, CT1{-0.06044}, CT2{-0.1079};   ///< thrust coefficient in the advance ratio
    real_t CQ0{0.005230}, CQ1{0.004970}, CQ2{-0.01664}; ///< torque coefficient in it

    // Control limits [rad]
    real_t max_elevator{0.5};
    real_t max_aileron{0.5};
    real_t max_rudder{0.5};
    real_t max_throttle{1.0}; ///< the throttle is in [0, max_throttle]

    // Landing gear and ground
    std::vector<LandingGearWheel> gear{
        {{0.55, 0.0, 0.32}, 2000.0, 120.0, true, false},
        {{-0.08, -0.45, 0.32}, 2000.0, 120.0, false, true},
        {{-0.08, 0.45, 0.32}, 2000.0, 120.0, false, true},
    };
    real_t nose_steering{1.0}; ///< nose wheel angle per rad of rudder
    real_t rolling_friction{0.03};
    real_t brake_friction{0.6}; ///< at full brake
    real_t side_friction{0.8};

    ///
    /// @brief The Aerosonde of Beard and McLain, Small Unmanned Aircraft, 2nd edition, with a
    /// tricycle landing gear
    ///
    static FixedWingParameters aerosonde() { return {}; }

    real_t aspect_ratio() const { return b * b / S; }
};

///
/// @brief The forces and moments on the aircraft, in the body frame, and the air data they come
/// from
///
struct FixedWingForces
{
    RealColVec3d force;      ///< [N]
    RealColVec3d moment;     ///< [N m]
    real_t airspeed{0.0};    ///< [m/s]
    real_t alpha{0.0};       ///< the angle of attack [rad]
    real_t beta{0.0};        ///< the sideslip angle [rad]
    real_t thrust{0.0};      ///< [N]
    real_t ground_load{0.0}; ///< the sum of the wheels' normal forces [N]
};

/**
 * @class FixedWingDynamics
 * @brief The 6-DOF flight dynamics of a fixed wing aircraft, with its landing gear on a flat
 * ground at down = 0.
 *
 * The world is NED and the body frame has x forward, y right and z down; the body is turned from
 * the world by \f$R = R_z(\psi) R_y(\theta) R_x(\phi)\f$. The model follows Beard and McLain,
 * Small Unmanned Aircraft: Theory and Practice:
 *
 * - **Air data.** The velocity relative to the air is \f$v_r = v - R^T w\f$, with \f$w\f$ the
 *   wind in the world, and gives the airspeed \f$V_a\f$, the angle of attack
 *   \f$\alpha = \arctan(w_r / u_r)\f$ and the sideslip \f$\beta = \arcsin(v_r / V_a)\f$.
 * - **Lift and drag** come from \f$\alpha\f$, the pitch rate and the elevator. The lift is linear
 *   in \f$\alpha\f$ up to the stall, then blends into that of a flat plate; the drag is
 *   \f$C_{D0}\f$ plus the induced drag \f$C_L^2 / (\pi e A)\f$.
 * - **Side force and the moments** are linear in \f$\beta\f$, the rates and the controls.
 * - **The propeller** is driven by a DC motor: for the throttle's voltage and the airspeed, the
 *   motor speed is where the motor's torque equals the propeller's, and the thrust and torque
 *   follow from the advance ratio.
 * - **Each wheel** pushes up with \f$k d + c \dot d\f$ while it is \f$d\f$ below the ground,
 *   and rolls and grips with friction proportional to that load.
 *
 * step() integrates with fourth order Runge-Kutta, in substeps of at most 2 ms for the stiff
 * landing gear.
 */
class FixedWingDynamics
{
  public:
    FixedWingDynamics(const FixedWingParameters &parameters, const FixedWingState &state);

    ///
    /// @brief Advance by dt [s] with the given controls, which are first limited
    ///
    void step(const FixedWingControls &controls, real_t dt);

    ///
    /// @return The derivative of the state, for the state and the controls
    ///
    FixedWingState derivative(const FixedWingState &x, const FixedWingControls &controls) const;

    ///
    /// @return The forces and moments on the aircraft, in the body frame
    ///
    FixedWingForces forces(const FixedWingState &x, const FixedWingControls &controls) const;

    ///
    /// @return The controls limited to what the aircraft can do
    ///
    FixedWingControls limit(const FixedWingControls &controls) const;

    ///
    /// @brief The wind, in the world [m/s]; the direction it blows towards
    ///
    void set_wind(const RealColVec3d &wind_ned) { wind_ = wind_ned; }
    const RealColVec3d &wind() const noexcept { return wind_; }

    const FixedWingState &state() const noexcept { return state_; }
    FixedWingState &state() noexcept { return state_; }
    const FixedWingParameters &parameters() const noexcept { return p_; }
    real_t time() const noexcept { return time_; }

    // The air data and the motion of the current state
    real_t airspeed() const;
    real_t alpha() const;
    real_t beta() const;
    real_t altitude() const { return -state_[fw::PD]; }
    real_t ground_speed() const;
    real_t course() const;     ///< the direction of the velocity over the ground [rad]
    real_t climb_rate() const; ///< [m/s], positive up
    bool on_ground() const;    ///< whether a wheel touches the ground

    ///
    /// @return The rotation from the body frame to the world
    ///
    static RealMat3d rotation(real_t phi, real_t theta, real_t psi);

  private:
    FixedWingParameters p_;
    FixedWingState state_;
    RealColVec3d wind_{RealColVec3d::Zero()};
    real_t time_{0.0};

    // Gamma terms of the rotational equations
    real_t g_[9];

    void propeller_(real_t Va, real_t throttle, real_t &thrust, real_t &torque) const;
};

} // namespace bitsim::dynamics

#endif // BITSIM_DYNAMICS_FIXED_WING_DYNAMICS_H

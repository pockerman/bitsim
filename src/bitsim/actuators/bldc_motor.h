#ifndef BITSIM_ACTUATORS_BLDC_MOTOR_H
#define BITSIM_ACTUATORS_BLDC_MOTOR_H

#include "bitsim/bitsim_types.h"

namespace bitsim
{
namespace actuators
{

///
/// @brief The parameters of a brushless DC motor. R and L are measured between two terminals,
/// which is what a datasheet gives. The defaults are a small motor of the size used in robot
/// joints
///
struct BLDCMotorParameters
{
    real_t R{0.6};         ///< resistance between two terminals [Ohm]
    real_t L{0.4e-3};      ///< inductance between two terminals [H]
    real_t Ke{0.05};       ///< back-EMF constant [V s/rad]; Kt = Ke in SI units
    real_t J{2.0e-5};      ///< inertia of the rotor [kg m^2]
    real_t b{1.0e-5};      ///< viscous friction [N m s/rad]
    int pole_pairs{7};     ///< electrical turns per mechanical turn
    real_t V_supply{24.0}; ///< supply voltage [V]

    ///
    /// @brief The torque constant [N m/A], equal to Ke in SI units
    ///
    real_t Kt() const noexcept { return Ke; }
};

///
/// @brief The state of the DC-equivalent model: current [A] and speed [rad/s]
///
struct DCEquivalentState
{
    real_t i{0.0};
    real_t w{0.0};
};

///
/// @brief One explicit Euler step of dt [s] of the DC-equivalent model: the motor seen from its
/// supply with the commutation averaged out, which is the model of a brushed DC motor:
///
/// \f[
/// L \frac{di}{dt} = V - R i - K_e \omega, \qquad
/// J \frac{d\omega}{dt} = K_t i - b \omega - T_{load}
/// \f]
///
DCEquivalentState dc_equivalent_step(const BLDCMotorParameters &m, const DCEquivalentState &s,
                                     real_t V, real_t T_load, real_t dt);

///
/// @brief The two phases that carry the current in a sector: it goes in through high and out
/// through low. The third phase is disconnected. Phases a, b, c are 0, 1, 2
///
struct PhasePair
{
    int high{0};
    int low{0};
};

/**
 * @class BLDCMotor
 * @brief The electrical model of a three-phase brushless motor with six-step commutation from
 * three Hall sensors.
 *
 * The rotor's angle and speed come from outside, so the same model works with a simple rotor
 * model, BLDCRotor, and with a rigid body in a physics engine. Each step works out the sector
 * from the Hall sensors, connects the two phases of the commutation table in series, advances
 * their current and returns the torque. The back-EMF of each phase is sinusoidal, scaled so
 * that its average over a sector matches the DC-equivalent model.
 */
class BLDCMotor
{
  public:
    ///
    /// @param m The motor
    /// @param hall_offset How far the Hall sensors are from where they should be [electrical
    /// rad]; a misplaced sensor makes the controller see the wrong sector
    ///
    explicit BLDCMotor(const BLDCMotorParameters &m, real_t hall_offset = 0.0);

    ///
    /// @brief The code of the three Hall sensors, 120 electrical degrees apart, at electrical
    /// angle theta_e: h1 + 2 h2 + 4 h3. Each sensor is on for half a turn, so the code tells
    /// which of six sectors the rotor is in
    ///
    static int hall_code(real_t theta_e);

    ///
    /// @brief The commutation table: for each Hall code the pair of phases whose back-EMF is
    /// largest in that sector. Codes 0 and 7 never happen with working sensors and give {0, 0}
    ///
    static PhasePair commutation(int hall_code);

    ///
    /// @brief The back-EMF of a phase per unit of mechanical speed [V s/rad]. The phases are
    /// shifted by 120 electrical degrees
    ///
    real_t phase_emf(int phase, real_t theta_e) const;

    ///
    /// @brief The electrical angle the controller sees for the mechanical angle theta [rad]
    ///
    real_t sensed_angle(real_t theta) const;

    ///
    /// @brief Advance the current by dt [s] for supply voltage V and return the torque [N m].
    /// theta and w are the mechanical angle [rad] and speed [rad/s] of the rotor. The current is
    /// taken to move to the new pair at once when the sector changes
    ///
    real_t step(real_t V, real_t theta, real_t w, real_t dt);

    real_t current() const noexcept { return i_; }
    const PhasePair &pair() const noexcept { return pair_; }

    ///
    /// @brief The peak of the line-to-line back-EMF per unit of speed: \f$\pi K_e / 3\f$
    ///
    real_t k_line() const noexcept { return k_line_; }

    const BLDCMotorParameters &parameters() const noexcept { return m_; }

  private:
    BLDCMotorParameters m_;
    real_t hall_offset_;
    real_t k_line_;
    real_t k_phase_;
    real_t i_{0.0};
    PhasePair pair_{0, 0};
};

///
/// @brief A rotor with inertia and viscous friction, for a BLDCMotor without a physics engine:
/// \f$J \dot\omega = T_{motor} - b \omega - T_{load}\f$
///
struct BLDCRotor
{
    real_t theta{0.0}; ///< mechanical angle [rad]
    real_t w{0.0};     ///< speed [rad/s]

    ///
    /// @brief One explicit Euler step of dt [s]
    ///
    void step(const BLDCMotorParameters &m, real_t T_motor, real_t T_load, real_t dt);
};

} // namespace actuators
} // namespace bitsim

#endif // BITSIM_ACTUATORS_BLDC_MOTOR_H

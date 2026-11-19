#ifndef DIFF_DRIVE_DYNAMICS_H
#define DIFF_DRIVE_DYNAMICS_H

#include "bitrl/bitrl_types.h"
#include "bitrl/dynamics/dynamics_matrix_descriptor.h"
#include "bitrl/dynamics/motion_model_base.h"
#include "bitrl/dynamics/system_state.h"

#include <any>
#include <array>
#include <map>

namespace bitrl
{
namespace dynamics
{

///
/// \brief DiffDriveDynamics class. Describes the motion of a differential drive robot in the
/// plane: the centre of its axle at (x, y), facing theta. Three versions:
///
/// - V1, the default: the input is the speed v [m/s] and the rate of turn w [rad/s], and over a
///   step the robot drives an arc of length \f$d = v \Delta t + e_0\f$ that turns it by
///   \f$\phi = \omega \Delta t + e_1\f$, where \f$e_0\f$ and \f$e_1\f$ are the `errors`:
///   \f[
///   x' = x + \frac{d}{\phi} (\sin\theta' - \sin\theta), \quad
///   y' = y - \frac{d}{\phi} (\cos\theta' - \cos\theta), \quad \theta' = \theta + \phi
///   \f]
///   With \f$|\phi|\f$ below the tolerance it moves by d along \f$\theta + \phi/2\f$, what
///   the arc tends to. With no errors this is exact for a constant v and w. update_matrices()
///   gives its Jacobians F, with respect to the state, and L, with respect to the errors, for
///   an extended Kalman filter. theta' is wrapped to \f$(-\pi, \pi]\f$.
/// - V2: a forward Euler step with v and w: the robot moves along its heading, then turns.
/// - V3: the speeds of the two wheels as input, w1 and w2, with the wheel radius r and l, and a
///   forward Euler step.
///
class DiffDriveDynamics : public MotionModelDynamicsBase<SysState<3>, DynamicsMatrixDescriptor,
                                                         std::map<std::string, std::any>>
{
  public:
    ///
    /// \brief The DynamicVersion enum. Helper enum to
    /// disambiguate the two supplied configurations
    ///
    enum class DynamicVersion
    {
        V1,
        V2,
        V3
    };

    ///
    /// \brief The type of the state handled by this dynamics object
    ///
    typedef MotionModelDynamicsBase<SysState<3>, DynamicsMatrixDescriptor,
                                    std::map<std::string, std::any>>::state_type state_type;

    ///
    /// \brief input_t The type of the input for solving the dynamics
    ///
    typedef MotionModelDynamicsBase<SysState<3>, DynamicsMatrixDescriptor,
                                    std::map<std::string, std::any>>::input_type input_type;

    ///
    /// \brief matrix_t Matrix type that describes the dynamics
    ///
    typedef MotionModelDynamicsBase<SysState<3>, DynamicsMatrixDescriptor,
                                    std::map<std::string, std::any>>::matrix_type matrix_type;

    ///
    /// \brief vector_t
    ///
    typedef MotionModelDynamicsBase<SysState<3>, DynamicsMatrixDescriptor,
                                    std::map<std::string, std::any>>::vector_type vector_type;

    ///
    /// \brief integrate_state_v1
    ///
    static SysState<3> integrate_state_v1(const SysState<3> &state, real_t tol, real_t dt, real_t v,
                                          real_t w, const std::array<real_t, 2> &errors);

    ///
    /// \brief integrate_state_v2
    ///
    static SysState<3> integrate_state_v2(const SysState<3> &state, real_t dt, real_t v, real_t w,
                                          const std::array<real_t, 2> &errors);

    ///
    /// \brief integrate_state_v3
    ///
    static SysState<3> integrate_state_v3(const SysState<3> &state, real_t r, real_t l, real_t dt,
                                          real_t w1, real_t w2,
                                          const std::array<real_t, 2> &errors);

    ///
    /// \brief integrate Factory method to apply
    /// the different integration methods
    ///
    static SysState<3> integrate(const SysState<3> &state, const input_type &input,
                                 const DynamicVersion version);

    ///
    /// \brief Constructor
    ///
    explicit DiffDriveDynamics(DynamicVersion type = DynamicVersion::V1,
                               bool update_description_matrices_on_evaluate = true);

    ///
    /// \brief Constructor
    ///
    explicit DiffDriveDynamics(state_type &&state);

    ///
    /// \brief Evaluate the new state using the given input
    /// it also updates the various matrices if needed
    ///
    virtual state_type &evaluate(const input_type &input) override;

    ///
    /// \brief Integrate the new state. It also uses error terms
    ///
    void integrate(const input_type &input);

    ///
    /// \brief Read the x-coordinate
    ///
    real_t get_x_position() const { return this->state_.get("X"); }

    ///
    /// \brief Set the x-coordinate
    ///
    void set_x_position(real_t x) { this->state_.set("X", x); }

    ///
    /// \brief Read the y-coordinate
    ///
    real_t get_y_position() const { return this->state_.get("Y"); }

    ///
    /// \brief Set the y-coordinate
    ///
    void set_y_position(real_t y) { this->state_.set("Y", y); }

    ///
    /// \brief Read the y-coordinate
    ///
    real_t get_orientation() const { return this->state_.get("Theta"); }

    ///
    /// \brief Set the orientation
    ///
    void set_orientation(real_t theta) { this->state_.set("Theta", theta); }

    ///
    /// \brief get_velocity Returns the velocity used for integration
    ///
    real_t get_velocity() const { return v_; }

    ///
    /// \brief get_velocity Returns the angular velocity used for integration
    ///
    real_t get_angular_velocity() const { return w_; }

    ///
    /// \brief updates the matrices used to describe this
    /// motion model
    ///
    void update_matrices(const input_type &input);

    ///
    /// \brief Initialize the matrices describing the
    /// the dynamics
    ///
    void initialize_matrices(const input_type &input);

  private:
    ///
    /// \brief v_ The velocity used for integration. Updated
    /// every time the application calls integrate
    ///
    real_t v_;

    ///
    /// \brief w_ The angular velocity used for integration
    ///
    real_t w_;

    ///
    /// \brief
    ///
    DynamicVersion type_;
};

} // namespace dynamics

} // namespace bitrl

#endif // DIFF_DRIVE_DYNAMICS_H

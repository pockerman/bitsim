#ifndef BITSIM_DYNAMICS_BICYCLE_VEHICLE_MODEL_DYNAMICS_H
#define BITSIM_DYNAMICS_BICYCLE_VEHICLE_MODEL_DYNAMICS_H

#include "bitsim/bitsim_types.h"
#include "bitsim/dynamics/dynamics_matrix_descriptor.h"
#include "bitsim/dynamics/motion_model_base.h"
#include "bitsim/dynamics/system_state.h"

#include <any>
#include <map>
#include <string>

namespace bitsim
{
namespace dynamics
{

struct BicycleVehicleModelDynamicsConfig
{
    ///
    /// \brief The time step to use in secs
    ///
    real_t dt;

    ///
    /// \brief The distance between the front and
    /// real wheels in m
    ///
    real_t L;
};

///
/// \brief The BicycleVehicleModelDynamics class.
/// Implements the bicycle vehicle model dynamics.
/// The implementation of the dynamics is taken
/// from the paper: Deep Reinforcement Learning for Mobile Robot Path Planning
/// by Hao Liu, Yi Shen, Shuangjiang Yu, Zijun Gao, Tong Wu
/// The state of the model is described by the tuple (X, Y, Phi)
/// where (X,Y) denotes the 2D position of the vehicle in the world coordinates
/// and phi denotes the heading of the vehicle
///
class BicycleVehicleModelDynamics
    : public MotionModelDynamicsBase<SysState<3>, DynamicsMatrixDescriptor,
                                     std::map<std::string, std::any>>
{

  public:
    typedef MotionModelDynamicsBase<SysState<3>, DynamicsMatrixDescriptor,
                                    std::map<std::string, std::any>>
        base_type;

    typedef base_type::state_type state_type;
    typedef base_type::input_type input_type;
    typedef base_type::matrix_descriptor_type matrix_descriptor_type;
    typedef typename matrix_descriptor_type::matrix_type matrix_type;
    typedef typename matrix_descriptor_type::vector_type vector_type;

    ///
    /// \brief Constructor
    ///
    BicycleVehicleModelDynamics(const BicycleVehicleModelDynamicsConfig config,
                                SysState<3> &init_state);

    ///
    /// \brief Move the state by one step of config.dt with the inputs "v", the speed [m/s],
    /// "delta", the steering angle of the front wheel [rad], and "errors", the errors added to
    /// both, an std::array<real_t, 2>:
    /// x += v cos(theta) dt, y += v sin(theta) dt, theta += v tan(delta) dt / L.
    /// If matrix updates are allowed, F and L are first set to the Jacobians of the step with
    /// respect to the state and to the errors
    ///
    void integrate(const input_type &input);

    ///
    /// \brief evaluate
    /// \param input
    /// \return
    ///
    state_type &evaluate(const input_type &input);

    ///
    /// \brief initialize_matrices. Initialize the matrix description of the
    /// dynamics
    ///
    void initialize_matrices(const input_type &input);

    ///
    /// \brief update_matrices. Update the matrix description of the
    /// dynamics
    ///
    void update_matrices(const input_type &input);

    ///
    /// \brief load_from_json Load the description of the chassis from
    /// the given json file
    ///
    void load_from_json(const std::string &filename){};

  private:
    ///
    /// \brief Configuration of the dynamics integrator
    ///
    BicycleVehicleModelDynamicsConfig config_;
};

} // namespace dynamics
} // namespace bitsim

#endif // BITSIM_DYNAMICS_BICYCLE_VEHICLE_MODEL_DYNAMICS_H

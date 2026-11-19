#ifndef BITRL_ESTIMATION_DIFF_DRIVE_EKF_H
#define BITRL_ESTIMATION_DIFF_DRIVE_EKF_H

#include "bitrl/bitrl_types.h"
#include "bitrl/dynamics/system_state.h"
#include "bitrl/estimation/extended_kalman_filter.h"
#include "bitrl/trajectory/rounded_rectangle.h"

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace bitrl
{
namespace estimation
{

///
/// @brief An axis-aligned rectangle in the plane [m]: a wall or a box, seen from above
///
struct Box2d
{
    real_t x_min{0.0};
    real_t y_min{0.0};
    real_t x_max{0.0};
    real_t y_max{0.0};
};

/**
 * @class DiffDriveMap
 * @brief What a robot knows about where it drives: the line on the floor, a RoundedRectangle,
 * and the walls and boxes its range sensors can hear.
 */
class DiffDriveMap
{
  public:
    DiffDriveMap(const trajectory::RoundedRectangle &track, std::vector<Box2d> obstacles);

    const trajectory::RoundedRectangle &track() const noexcept { return track_; }
    const std::vector<Box2d> &obstacles() const noexcept { return obstacles_; }

    ///
    /// @brief How far a ray from (x, y) at angle [rad] from the x axis goes before it meets an
    /// obstacle, at most max_range [m]
    ///
    real_t ray_cast(real_t x, real_t y, real_t angle, real_t max_range) const;

  private:
    trajectory::RoundedRectangle track_;
    std::vector<Box2d> obstacles_;
};

/**
 * @class DiffDriveGyroModel
 * @brief The motion model of DiffDriveEKF: a differential drive robot that knows how far it drove
 * from its wheel encoders and how fast it turned from a gyroscope with a bias.
 *
 * The state is (x, y, yaw, b): the centre of the axle, the heading and the gyroscope's bias.
 * Over a step of dt the robot drives d and turns by \f$\Delta\psi = (g - b)\, dt\f$, where g is
 * what the gyroscope reads:
 *
 * \f[
 * x' = x + d \cos(\psi + \Delta\psi / 2), \quad y' = y + d \sin(\psi + \Delta\psi / 2), \quad
 * \psi' = \psi + \Delta\psi, \quad b' = b
 * \f]
 *
 * The process noise is (error in d, noise of the gyroscope, random walk of the bias), and F and L
 * are the Jacobians of the step with respect to the state and to that noise.
 */
class DiffDriveGyroModel
{
  public:
    typedef dynamics::SysState<4> state_type;
    typedef DynMat<real_t> matrix_type;

    struct Input
    {
        real_t distance{0.0}; ///< driven over the step, from the encoders [m]
        real_t gyro_z{0.0};   ///< the gyroscope's reading about the vertical [rad/s]
        real_t dt{0.0};       ///< the step [s]
    };
    typedef Input input_type;

    DiffDriveGyroModel();

    ///
    /// @brief Move the state by one step, and work out F and L at the state before it
    ///
    state_type &evaluate(const Input &input);

    state_type &get_state() { return state_; }
    const state_type &get_state() const { return state_; }
    real_t get_state_property(const std::string &name) const { return state_.get(name); }
    const matrix_type &get_matrix(const std::string &name) const;

  private:
    state_type state_;
    matrix_type F_;
    matrix_type L_;
};

/**
 * @class DiffDriveEKF
 * @brief An extended Kalman filter that fuses everything a small differential drive robot
 * senses: wheel encoders, a gyroscope, infrared elements and a camera that see a line on the
 * floor, and ultrasonic range sensors.
 *
 * The encoders and the gyroscope predict, see DiffDriveGyroModel; on their own they drift, as
 * dead reckoning does. The other sensors correct, against a DiffDriveMap:
 *
 * - **the line**: a point where the infrared elements or the camera see the line, relative to the
 *   robot, should be on the track. Its signed distance to the track, \f$h(x)\f$, is measured as 0.
 *   Across the line this fixes the position, and with points at different distances ahead, the
 *   camera's, the heading too; along the line it says nothing;
 * - **ranges**: an ultrasonic sensor at a known place on the robot should measure the shortest of
 *   its rays to the walls and boxes of the map, cast across its beam as the sensor hears them.
 *   Ahead and behind along a straight this fixes the position along the track.
 *
 * Every update goes through the gate of ExtendedKalmanFilter: a measurement that the estimate
 * cannot explain, such as an echo from something not on the map, is not used. The Jacobians of the
 * measurements are worked out by central differences.
 */
class DiffDriveEKF
{
  public:
    struct Parameters
    {
        real_t distance_noise{0.01}; ///< of the encoders' distance [m per square root of m]
        real_t gyro_noise{0.002};    ///< of each reading of the gyroscope [rad/s]
        real_t bias_walk{1.0e-4};    ///< how fast the bias may wander [rad/s per square root of s]
        real_t line_noise{0.015};    ///< of a point of the line [m]
        real_t range_noise{0.03};    ///< of an ultrasonic range [m]
        real_t gate{9.0};            ///< per measured value, on the squared Mahalanobis distance
    };

    struct Pose
    {
        real_t x{0.0};
        real_t y{0.0};
        real_t yaw{0.0};
    };

    ///
    /// @brief Where an ultrasonic sensor is on the robot, relative to the centre of the axle,
    /// and what it hears
    ///
    struct RangeSensor
    {
        real_t x{0.0};          ///< ahead of the axle [m]
        real_t y{0.0};          ///< to the left [m]
        real_t yaw{0.0};        ///< where it points, from straight ahead [rad]
        real_t beam_angle{0.0}; ///< the full width of its cone [rad]
        uint_t n_rays{1};       ///< how many rays its model casts across the cone
        real_t min_range{0.02}; ///< [m]
        real_t max_range{4.0};  ///< [m]; a reading this far is no echo
    };

    ///
    /// @param map What the robot knows of its world
    /// @param start Where it starts, as it is placed
    /// @param parameters The noises and the gate
    /// @param start_sigma The standard deviations of the start: position [m], heading [rad] and
    /// gyroscope bias [rad/s]
    ///
    DiffDriveEKF(const DiffDriveMap &map, const Pose &start, const Parameters &parameters,
                 const std::array<real_t, 3> &start_sigma = {0.02, 0.02, 0.005});

    ///
    /// @brief Predict over dt [s] in which the encoders say the robot drove distance [m] and the
    /// gyroscope read gyro_z [rad/s]
    ///
    void predict(real_t distance, real_t gyro_z, real_t dt);

    ///
    /// @brief Correct with points where the line is, relative to the centre of the axle [m],
    /// forward and to the left, each with the standard deviation sigma [m]
    ///
    EKFUpdateResult update_line(const std::vector<std::array<real_t, 2>> &points, real_t sigma);

    ///
    /// @brief Correct with the reading [m] of a range sensor. A reading at the sensor's
    /// max_range, no echo, or where the map has nothing in range, is not used
    ///
    EKFUpdateResult update_range(const RangeSensor &sensor, real_t measured);

    ///
    /// @brief What the sensor should read with the robot at pose
    ///
    real_t predicted_range(const RangeSensor &sensor, const Pose &pose) const;

    Pose pose() const;
    real_t gyro_bias() const { return model_.get_state()[3]; }

    ///
    /// @brief The covariance of (x, y, yaw, bias)
    ///
    const DynMat<real_t> &covariance() const { return ekf_["P"]; }

    const Parameters &parameters() const noexcept { return p_; }
    const DiffDriveMap &map() const noexcept { return map_; }

  private:
    void wrap_heading_();

    DiffDriveMap map_;
    Parameters p_;
    DiffDriveGyroModel model_;
    ExtendedKalmanFilter<DiffDriveGyroModel> ekf_;
};

} // namespace estimation
} // namespace bitrl

#endif // BITRL_ESTIMATION_DIFF_DRIVE_EKF_H

#ifndef DIFF_DRIVE_ROBOT_H
#define DIFF_DRIVE_ROBOT_H

#include "bitrl/bitrl_config.h"

#ifdef BITRL_CHRONO

#include "bitrl/actuators/dc_motor.h"
#include "bitrl/bitrl_types.h"
#include "bitrl/extern/nlohmann/json/json.hpp"
#include "bitrl/rigid_bodies/chrono_robots/chrono_robot_pose.h"
#include "bitrl/sensors/sensor_base.h"
#include "bitrl/sensors/sensor_manager.h"

#include <chrono/functions/ChFunctionConst.h>
#include <chrono/functions/ChFunctionSetpoint.h>
#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChLinkLock.h>
#include <chrono/physics/ChLinkMotorRotation.h>
#include <chrono/physics/ChSystemNSC.h>

#include <array>
#include <filesystem>
#include <memory>

namespace bitrl
{
namespace rb::bitrl_chrono
{

/**
 * @class CHRONO_DiffDriveRobot
 * @ingroup rb_chrono
 * @brief A differential-drive robot in Chrono: a chassis, two driven wheels
 * and a ball caster. Each driven wheel has an actuators::DCMotor.
 *
 * How a motor turns its wheel depends on its backend. A simulated motor
 * produces a torque, and Chrono moves the wheel. A real motor on a board has
 * already turned, so Chrono turns the wheel to the angle it reports, and the
 * contact with the ground moves the robot. Either way, call update(dt) before
 * every DoStepDynamics(dt).
 *
 * The motors come from the "motors" section of the robot file. Without one, or
 * to use real motors, attach them with set_motor() before or after add_to_sys().
 */
class CHRONO_DiffDriveRobot
{
  public:
    using json = nlohmann::json;

    ///
    /// @brief The wheels, in the order of the motors
    ///
    enum Wheel : uint_t
    {
        LEFT = 0,
        RIGHT = 1
    };

    ///
    /// Helper class to assemble some robot data
    ///
    struct RobotData
    {
        real_t max_speed;           ///< the largest speed of a wheel on the ground [m/s]
        real_t wheel_radius;        ///< [m]
        real_t wheel_width;         ///< [m]
        real_t axle_length;         ///< the distance between the wheels [m]
        real_t density;             ///< [kg/m^3]
        real_t caster_wheel_radius; ///< [m]

        template <typename Archive> void read(const Archive &arch);
    };

    ///
    /// Class to manage the sensors attached on this
    ///
    class RobotSensorManager
    {

      public:
        ///
        /// Constructor
        ///
        RobotSensorManager();

        /// Load the sensors of the robot from the archive
        /// @tparam Archive
        /// @param arch
        template <typename Archive>
        void read(const Archive &arch, std::shared_ptr<chrono::ChBody> body);

        ///
        /// @return The number of registered senosrs
        uint_t n_sensors() const noexcept { return sensor_manager_.n_sensors(); }

        ///
        /// @brief Add a new sensor to manage
        /// @param sensor
        ///
        void add(std::shared_ptr<sensors::SensorBase> sensor) { sensor_manager_.add(sensor); }

      private:
        bitrl::sensors::SensorManager sensor_manager_;
    };

    ///
    /// @brief Constructor
    ///
    CHRONO_DiffDriveRobot() = default;

    ///
    /// @brief Load the robot details form the specified filename
    ///
    void load(const std::filesystem::path &filename);

    ///
    /// @brief Load the robot details form the specified json structure
    ///
    void load(const json &j);

    ///
    /// @return The number of wheel motors this robot has
    ///
    uint_t n_wheel_motors() const noexcept { return 2; }

    ///
    /// @brief Replace the motor of a wheel. The wheel is reconnected to the
    /// chassis to match the backend of the motor
    ///
    void set_motor(uint_t m, std::shared_ptr<actuators::DCMotor> motor);

    ///
    /// @return The motor of a wheel. m = 0 is the left and m = 1 the right
    ///
    std::shared_ptr<actuators::DCMotor> get_motor(uint_t m) const;

    ///
    /// @brief Set the target speed of the m-th wheel [rad/s]
    /// m=0 => left motor
    /// m=1 => right motor
    ///
    void set_motor_speed(uint_t m, real_t speed);

    ///
    /// @brief Set both wheels to the same target speed [rad/s]
    ///
    void set_motor_speed(real_t speed);

    ///
    /// @brief Drive straight at the given speed [m/s]
    ///
    void set_speed(real_t speed);

    ///
    /// @brief Drive with a forward speed v [m/s] and a turning rate omega
    /// [rad/s], positive to the left
    ///
    void set_velocity(real_t v, real_t omega);

    ///
    /// @brief Advance the motors by dt and pass their output to the wheels.
    /// Call it before every DoStepDynamics(dt)
    ///
    void update(real_t dt);

    ///
    /// @return The angle of a wheel relative to the chassis, as Chrono has it [rad]
    ///
    real_t wheel_angle(uint_t m) const;

    ///
    /// @return The torque on a wheel, as Chrono has it [N m]
    ///
    real_t wheel_torque(uint_t m) const;

    ///
    /// Get the pose of the robot
    ///
    const CHRONO_RobotPose &get_pose() const { return pose_; }

    ///
    /// @return The chassis of the robot
    ///
    std::shared_ptr<chrono::ChBody> get_chassis() const { return chassis_; }

    ///
    /// @return A wheel of the robot. m = 0 is the left and m = 1 the right
    ///
    std::shared_ptr<chrono::ChBody> get_wheel(uint_t m) const;

    ///
    /// @return The point halfway between the wheels, in world coordinates. The
    /// robot turns on the spot around it
    ///
    chrono::ChVector3d axle_center() const;

    ///
    /// @return The data read from the robot file
    ///
    const RobotData &get_data() const noexcept { return data_; }

    ///
    /// Add the components of this robot to the given chrono::ChSystemNSC
    /// @param sys
    void add_to_sys(chrono::ChSystemNSC &sys) const;

    /// Read/write reference to the senosr manager for this robot
    /// @return
    ///
    RobotSensorManager &get_sensor_manager() { return sensor_manager_; }

    /// @brief Explicitly set the world position of the robot. This is useful
    /// for changing the initial position of the robot but avoid using it within
    /// a simualtion running unless you know what you are doing
    /// @param position
    ///
    void set_world_position(const chrono::ChVector3d &position);

    /// @brief Explicitly set the world position of the robot. This is useful
    /// for changing the initial position of the robot but avoid using it within
    /// a simualtion running unless you know what you are doing
    /// @param position
    ///
    void set_world_position(const std::array<real_t, 3> &position);

  private:
    void check_loaded_() const;
    void check_motor_index_(uint_t m) const;
    void build_wheel_link_(uint_t m);

    /// the components of the robot
    std::shared_ptr<chrono::ChBodyEasyBox> chassis_;
    std::array<std::shared_ptr<chrono::ChBodyEasyCylinder>, 2> wheels_;
    std::array<chrono::ChVector3d, 2> wheel_positions_;
    std::shared_ptr<chrono::ChBodyEasySphere> caster_wheel_;
    std::shared_ptr<chrono::ChLinkLockSpherical> caster_link_joint_;

    /// The motors and how they connect the wheels to the chassis
    std::array<std::shared_ptr<actuators::DCMotor>, 2> motors_;
    std::array<std::shared_ptr<chrono::ChLinkMotorRotation>, 2> wheel_links_;
    std::array<std::shared_ptr<chrono::ChFunctionConst>, 2> wheel_torques_;
    std::array<std::shared_ptr<chrono::ChFunctionSetpoint>, 2> wheel_angles_;
    std::array<real_t, 2> angle_offsets_{0.0, 0.0};

    /// The pose of the chassis
    CHRONO_RobotPose pose_;
    RobotData data_;
    RobotSensorManager sensor_manager_;

    bool is_loaded_{false};
};
} // namespace rb::bitrl_chrono
} // namespace bitrl

#endif

#endif // DIFF_DRIVE_ROBOT_H

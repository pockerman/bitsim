#include "bitrl/rigid_bodies/chrono_robots/diff_drive_robot.h"

#ifdef BITRL_CHRONO

#include "bitrl/actuators/backends/sim_dc_motor_backend.h"
#include "bitrl/extern/nlohmann/json/json.hpp"
#include "bitrl/sensors/ultrasonic_sensor.h"

#include <chrono/collision/ChCollisionShapeSphere.h>
#include <chrono/physics/ChContactMaterialNSC.h>
#include <chrono/physics/ChLinkMotorRotationAngle.h>
#include <chrono/physics/ChLinkMotorRotationTorque.h>

#include <any>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace bitrl
{
namespace rb::bitrl_chrono
{
namespace
{

using json = nlohmann::json;

// The parts of the robot never collide with each other. The wheels overlap the
// chassis, and the links keep the parts where they belong anyway
const int ROBOT_COLLISION_FAMILY = 1;

void join_robot_family(chrono::ChBody &body)
{
    body.EnableCollision(true);
    body.GetCollisionModel()->SetFamily(ROBOT_COLLISION_FAMILY);
    body.GetCollisionModel()->DisallowCollisionsWith(ROBOT_COLLISION_FAMILY);
}

chrono::ChVector3d read_position(const json &j)
{
    return chrono::ChVector3d(j.at("pos_x").get<real_t>(), j.at("pos_y").get<real_t>(),
                              j.at("pos_z").get<real_t>());
}

auto build_material(const json &j)
{
    auto material = chrono_types::make_shared<chrono::ChContactMaterialNSC>();
    material->SetFriction(j.at("friction").get<float>());
    material->SetRestitution(j.at("restitution").get<float>());
    return material;
}

auto build_chassis(std::shared_ptr<chrono::ChContactMaterialNSC> material, const json &j,
                   const CHRONO_DiffDriveRobot::RobotData &data)
{
    const real_t x = j.at("chassis_x").get<real_t>();
    const real_t y = j.at("chassis_y").get<real_t>();
    const real_t z = j.at("chassis_z").get<real_t>();
    auto chassis = chrono_types::make_shared<chrono::ChBodyEasyBox>(x, y, z, data.density, true,
                                                                    true, material);

    // The chassis sits just above the wheel axles
    const auto pos = read_position(j);
    chassis->SetPos(chrono::ChVector3d(pos.x(), pos.y(), data.wheel_radius + z / 2.0 + 0.01));
    join_robot_family(*chassis);
    return chassis;
}

auto build_wheel(std::shared_ptr<chrono::ChContactMaterialNSC> material,
                 const chrono::ChVector3d &pos, const CHRONO_DiffDriveRobot::RobotData &data)
{
    // The wheel looks like a cylinder, but touches the ground like a sphere of
    // the same radius: one contact point instead of a ring of them, which makes
    // the contact problem much cheaper to solve
    auto wheel = chrono_types::make_shared<chrono::ChBodyEasyCylinder>(
        chrono::ChAxis::Y, data.wheel_radius, data.wheel_width, data.density, true, false);
    wheel->AddCollisionShape(
        chrono_types::make_shared<chrono::ChCollisionShapeSphere>(material, data.wheel_radius));
    wheel->SetPos(pos);
    join_robot_family(*wheel);
    return wheel;
}

auto build_caster_wheel(std::shared_ptr<chrono::ChContactMaterialNSC> material,
                        const chrono::ChVector3d &pos, const CHRONO_DiffDriveRobot::RobotData &data)
{
    auto caster = chrono_types::make_shared<chrono::ChBodyEasySphere>(
        data.caster_wheel_radius, data.density, true, true, material);
    caster->SetPos(pos);
    join_robot_family(*caster);
    return caster;
}

// The wheel motors as the robot file describes them. Without a description
// the robot gets two simulated motors with the default constants
std::shared_ptr<actuators::DCMotor> build_motor(const json &j, const std::string &name)
{
    actuators::backends::DCMotorParameters parameters;
    if (j.is_null())
    {
        return actuators::DCMotor::create_simulated(parameters, name);
    }

    const auto backend = j.value("backend", actuators::backends::SIM_DCMotorBackend::BACKEND_TYPE);
    if (backend != actuators::backends::SIM_DCMotorBackend::BACKEND_TYPE)
    {
        throw std::logic_error("The robot file can only describe motors with the " +
                               actuators::backends::SIM_DCMotorBackend::BACKEND_TYPE +
                               " backend. Attach other motors with set_motor()");
    }

    if (j.contains("parameters"))
    {
        parameters.read(j.at("parameters"));
    }
    return actuators::DCMotor::create_simulated(parameters, name);
}

struct RangeSensorData
{
    uint_t idx;
    bool enable;
    real_t update_rate;
    real_t max_distance;
    real_t min_distance;
    std::string backend;
    std::string name;
    std::vector<float_t> color;
    std::vector<float_t> frame_pos;
    std::vector<float_t> dimensions;

    RangeSensorData(const json &data)
        : idx(data.at("idx").get<uint_t>()), enable(data.at("enable").get<bool>()),
          update_rate(data.at("update_rate").get<real_t>()),
          max_distance(data.at("max_distance").get<real_t>()),
          min_distance(data.at("min_distance").get<real_t>()),
          backend(data.at("backend").get<std::string>()), name(data.at("name").get<std::string>()),
          color(data.at("color").get<std::vector<float_t>>()),
          frame_pos(data.at("frame_pos").get<std::vector<float_t>>()),
          dimensions(data.at("dimensions").get<std::vector<float_t>>())
    {
    }
};

} // namespace

template <> void CHRONO_DiffDriveRobot::RobotData::read(const json &j)
{
    max_speed = j.at("max_speed").get<real_t>();
    wheel_radius = j.at("wheel_radius").get<real_t>();
    wheel_width = j.at("wheel_width").get<real_t>();
    axle_length = j.at("axle_length").get<real_t>();
    density = j.at("density").get<real_t>();
    caster_wheel_radius = j.at("caster_wheel_radius").get<real_t>();
}

CHRONO_DiffDriveRobot::RobotSensorManager::RobotSensorManager() : sensor_manager_(0) {}

template <>
void CHRONO_DiffDriveRobot::RobotSensorManager::read(const json &j,
                                                     std::shared_ptr<chrono::ChBody> body)
{

    auto sensors = j.at("sensors");
    auto fsdata = RangeSensorData(sensors.at("front-sonar"));

    auto fs = sensors::UltrasonicSensor::create(fsdata.backend, fsdata.name);
    sensor_manager_.add(fs);

    // get the backend and create the visual shape
    auto backend_ptr = fs->backend_ptr();

    if (!backend_ptr)
    {
        throw std::runtime_error("Backend ptr pointer is not set");
    }

    backend_ptr->set_chrono_body(body);
    chrono::ChFrame<> sensor_frame(
        chrono::ChVector3d(fsdata.frame_pos[0], fsdata.frame_pos[1], fsdata.frame_pos[2]));

    chrono::ChColor color(fsdata.color[0], fsdata.color[1], fsdata.color[2]);
    std::unordered_map<std::string, std::any> vis_shape_properties;
    vis_shape_properties["chrono::ChColor"] = color;
    vis_shape_properties["chrono::ChFrame<>"] = sensor_frame;
    vis_shape_properties["x_length"] = fsdata.dimensions[0];
    vis_shape_properties["y_length"] = fsdata.dimensions[1];
    vis_shape_properties["z_length"] = fsdata.dimensions[2];
    backend_ptr->add_visual_shape(vis_shape_properties);
}

void CHRONO_DiffDriveRobot::load(const std::filesystem::path &filename)
{
    std::ifstream file(filename);
    if (!file)
    {
        throw std::runtime_error("Could not open the robot file " + filename.string());
    }
    load(json::parse(file));
}

void CHRONO_DiffDriveRobot::load(const json &j)
{
    if (is_loaded_)
    {
        return;
    }

    data_.read(j.at("robot-data"));
    auto material = build_material(j.at("material"));

    chassis_ = build_chassis(material, j.at("chassis"), data_);

    wheel_positions_[LEFT] = read_position(j.at("left-wheel"));
    wheel_positions_[RIGHT] = read_position(j.at("right-wheel"));
    for (uint_t m = 0; m < 2; ++m)
    {
        wheels_[m] = build_wheel(material, wheel_positions_[m], data_);
    }

    // A ball caster turns freely about every axis
    const auto caster_position = read_position(j.at("caster-wheel"));
    caster_wheel_ = build_caster_wheel(material, caster_position, data_);
    caster_link_joint_ = chrono_types::make_shared<chrono::ChLinkLockSpherical>();
    caster_link_joint_->Initialize(caster_wheel_, chassis_,
                                   chrono::ChFrame<>(caster_position, chrono::QUNIT));

    const json motors = j.contains("motors") ? j.at("motors") : json();
    motors_[LEFT] = build_motor(motors, "left");
    motors_[RIGHT] = build_motor(motors, "right");
    for (uint_t m = 0; m < 2; ++m)
    {
        motors_[m]->set_max_speed(data_.max_speed / data_.wheel_radius);
        build_wheel_link_(m);
    }

    // track the chassis for pose reports
    pose_.set_body(chassis_);

    // read the sensors
    sensor_manager_.read(j.at("sensors"), chassis_);

    is_loaded_ = true;
}

void CHRONO_DiffDriveRobot::build_wheel_link_(uint_t m)
{
    // The z axis of the link is the axle, pointing to the left of the robot,
    // so a positive angle rolls the wheel forward
    const chrono::ChFrame<> frame(wheels_[m]->GetPos(), chrono::QuatFromAngleX(-chrono::CH_PI_2));

    std::shared_ptr<chrono::ChLinkMotorRotation> link;
    if (motors_[m]->output_type() == actuators::DCMotor::OutputType::TORQUE)
    {
        // The motor model gives a torque and Chrono turns the wheel
        wheel_torques_[m] = chrono_types::make_shared<chrono::ChFunctionConst>(0.0);
        auto torque_link = chrono_types::make_shared<chrono::ChLinkMotorRotationTorque>();
        torque_link->SetTorqueFunction(wheel_torques_[m]);
        link = torque_link;
    }
    else
    {
        // The motor has already turned, so the wheel follows its angle
        wheel_angles_[m] = chrono_types::make_shared<chrono::ChFunctionSetpoint>();
        auto angle_link = chrono_types::make_shared<chrono::ChLinkMotorRotationAngle>();
        angle_link->SetAngleFunction(wheel_angles_[m]);
        link = angle_link;
        angle_offsets_[m] = motors_[m]->angle();
    }
    link->Initialize(wheels_[m], chassis_, frame);

    // Swap the links if the robot is already simulated
    if (auto *system = chassis_->GetSystem())
    {
        if (wheel_links_[m])
        {
            system->RemoveLink(wheel_links_[m]);
        }
        system->AddLink(link);
    }
    wheel_links_[m] = link;
}

void CHRONO_DiffDriveRobot::add_to_sys(chrono::ChSystemNSC &sys) const
{
    check_loaded_();

    sys.Add(chassis_);
    sys.Add(wheels_[LEFT]);
    sys.Add(wheels_[RIGHT]);
    sys.Add(wheel_links_[LEFT]);
    sys.Add(wheel_links_[RIGHT]);
    sys.Add(caster_wheel_);
    sys.Add(caster_link_joint_);
}

void CHRONO_DiffDriveRobot::set_motor(uint_t m, std::shared_ptr<actuators::DCMotor> motor)
{
    check_loaded_();
    check_motor_index_(m);
    if (!motor)
    {
        throw std::logic_error("CHRONO_DiffDriveRobot::set_motor needs a motor");
    }

    motors_[m] = motor;
    motors_[m]->set_max_speed(data_.max_speed / data_.wheel_radius);
    build_wheel_link_(m);
}

std::shared_ptr<actuators::DCMotor> CHRONO_DiffDriveRobot::get_motor(uint_t m) const
{
    check_loaded_();
    check_motor_index_(m);
    return motors_[m];
}

void CHRONO_DiffDriveRobot::set_motor_speed(uint_t m, real_t speed)
{
    check_loaded_();
    check_motor_index_(m);
    motors_[m]->set_speed(speed);
}

void CHRONO_DiffDriveRobot::set_motor_speed(real_t speed)
{
    set_motor_speed(LEFT, speed);
    set_motor_speed(RIGHT, speed);
}

void CHRONO_DiffDriveRobot::set_speed(real_t speed) { set_velocity(speed, 0.0); }

void CHRONO_DiffDriveRobot::set_velocity(real_t v, real_t omega)
{
    check_loaded_();

    // The wheels move at v -/+ omega L / 2 along the ground
    const real_t half_axle = data_.axle_length / 2.0;
    set_motor_speed(LEFT, (v - omega * half_axle) / data_.wheel_radius);
    set_motor_speed(RIGHT, (v + omega * half_axle) / data_.wheel_radius);
}

void CHRONO_DiffDriveRobot::update(real_t dt)
{
    check_loaded_();

    const auto *system = chassis_->GetSystem();
    const real_t time = system ? system->GetChTime() : 0.0;

    for (uint_t m = 0; m < 2; ++m)
    {
        auto &motor = motors_[m];
        const auto &link = wheel_links_[m];

        if (motor->output_type() == actuators::DCMotor::OutputType::TORQUE)
        {
            motor->step(dt, actuators::ShaftState{link->GetMotorAngle(), link->GetMotorAngleDt()});
            wheel_torques_[m]->SetConstant(motor->torque());
        }
        else
        {
            motor->step(dt);
            wheel_angles_[m]->SetSetpoint(motor->angle() - angle_offsets_[m], time + dt);
        }
    }
}

std::shared_ptr<chrono::ChBody> CHRONO_DiffDriveRobot::get_wheel(uint_t m) const
{
    check_loaded_();
    check_motor_index_(m);
    return wheels_[m];
}

chrono::ChVector3d CHRONO_DiffDriveRobot::axle_center() const
{
    check_loaded_();
    return 0.5 * (wheels_[LEFT]->GetPos() + wheels_[RIGHT]->GetPos());
}

real_t CHRONO_DiffDriveRobot::wheel_angle(uint_t m) const
{
    check_loaded_();
    check_motor_index_(m);
    return wheel_links_[m]->GetMotorAngle();
}

real_t CHRONO_DiffDriveRobot::wheel_torque(uint_t m) const
{
    check_loaded_();
    check_motor_index_(m);
    return wheel_links_[m]->GetMotorTorque();
}

void CHRONO_DiffDriveRobot::set_world_position(const chrono::ChVector3d &position)
{
    chassis_->SetPos(position);
}

void CHRONO_DiffDriveRobot::set_world_position(const std::array<real_t, 3> &position)
{
    chrono::ChVector3d pos(position[0], position[1], position[2]);
    set_world_position(pos);
}

void CHRONO_DiffDriveRobot::check_loaded_() const
{
    if (!is_loaded_)
    {
        throw std::logic_error("Robot has not been loaded have you called load?");
    }
}

void CHRONO_DiffDriveRobot::check_motor_index_(uint_t m) const
{
    if (m != LEFT && m != RIGHT)
    {
        throw std::logic_error("Invalid motor index. Index must be either 0 or 1");
    }
}

} // namespace rb::bitrl_chrono
} // namespace bitrl

#endif

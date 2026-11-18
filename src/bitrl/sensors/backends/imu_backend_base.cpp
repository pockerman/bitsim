#include "bitrl/sensors/backends/imu_backend_base.h"
#include "bitrl/bitrl_config.h"
#include "bitrl/sensors/backends/chrono_imu_backend.h"
#include "bitrl/sensors/backends/mqtt_imu_backend.h"
#include "bitrl/utils/std_map_utils.h"

#ifdef BITRL_CHRONO
#include <chrono/assets/ChVisualShapeBox.h>
#endif

#include <fstream>
#include <stdexcept>

namespace bitrl
{
namespace sensors::backends
{

std::shared_ptr<IMUBackendBase> IMUBackendBase::create(const std::string &backend_type)
{
    if (backend_type == "SIMULATION" || backend_type == "CHRONO")
    {
#ifdef BITRL_CHRONO
        return std::make_shared<CHRONO_IMUBackend>();
#else
        throw std::runtime_error("The " + backend_type +
                                 " IMU backend needs bitrl built with Chrono");
#endif
    }

    if (backend_type == "MQTT")
    {
#ifdef BITRL_MQTT
        return std::make_shared<MQTT_IMUBackend>();
#else
        throw std::runtime_error("The MQTT IMU backend needs bitrl built with MQTT");
#endif
    }

    throw std::runtime_error("Unknown IMU backend type: " + backend_type);
}

void IMUBackendBase::load(const nlohmann::json &j)
{
    if (j.contains("frame_pos"))
    {
        const auto &p = j.at("frame_pos");
        position_ = {p.at(0).get<real_t>(), p.at(1).get<real_t>(), p.at(2).get<real_t>()};
    }

    if (j.contains("update_rate"))
    {
        const auto rate = j.at("update_rate").get<real_t>();
        if (rate <= 0.0)
        {
            throw std::logic_error("An IMU needs a positive update_rate");
        }
        set_sampling_period(1.0 / rate);
    }
}

void IMUBackendBase::load_from_json(const std::string &filename)
{
    std::ifstream file(filename);
    if (!file)
    {
        throw std::runtime_error("Could not open the sensor file " + filename);
    }
    load(nlohmann::json::parse(file));
}

#ifdef BITRL_CHRONO
void IMUBackendBase::add_visual_shape(
    const std::unordered_map<std::string, std::any> &shape_properties)
{
    if (!body_)
    {
        throw std::logic_error("The IMU has no body: call set_chrono_body()");
    }

    const auto x_length = utils::resolve<float_t>("x_length", shape_properties);
    const auto y_length = utils::resolve<float_t>("y_length", shape_properties);
    const auto z_length = utils::resolve<float_t>("z_length", shape_properties);
    const auto color = utils::resolve<chrono::ChColor>("chrono::ChColor", shape_properties);

    auto shape = chrono_types::make_shared<chrono::ChVisualShapeBox>(x_length, y_length, z_length);
    shape->SetColor(color);
    body_->AddVisualShape(
        shape, chrono::ChFrame<>(chrono::ChVector3d(position_[0], position_[1], position_[2])));
}
#endif

} // namespace sensors::backends
} // namespace bitrl

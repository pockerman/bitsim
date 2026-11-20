#include "bitsim/sensors/backends/range_sensor_backend_base.h"
#include "bitsim/bitsim_config.h"
#include "bitsim/sensors/backends/chrono_ultrasound_backend.h"
#include "bitsim/sensors/backends/mqtt_ultrasound_backend.h"
#include "bitsim/utils/std_map_utils.h"

#ifdef BITSIM_CHRONO
#include <chrono/assets/ChVisualShapeBox.h>
#endif

#include <algorithm>
#include <fstream>
#include <stdexcept>
namespace bitsim
{
namespace sensors::backends
{
std::shared_ptr<RangeSensorBackendBase>
RangeSensorBackendBase::create(const std::string &backend_type)
{
    if (backend_type == "SIMULATION" || backend_type == "CHRONO")
    {
#ifdef BITSIM_CHRONO
        return std::make_shared<CHRONO_UltrasonicBackend>();
#else
        throw std::runtime_error("The " + backend_type +
                                 " range sensor backend needs bitsim built with Chrono");
#endif
    }

    if (backend_type == "MQTT")
    {
#ifdef BITSIM_MQTT
        return std::make_shared<MQTT_UltrasonicBackend>();
#else
        throw std::runtime_error("The MQTT range sensor backend needs bitsim built with MQTT");
#endif
    }

    throw std::runtime_error("Unknown range sensor backend type: " + backend_type);
}

void RangeSensorBackendBase::load(const nlohmann::json &j)
{
    set_max_distance(j.value("max_distance", max_distance()));
    set_min_distance(j.value("min_distance", min_distance()));
    if (min_distance() < 0.0 || max_distance() < min_distance())
    {
        throw std::logic_error("A range sensor needs 0 <= min_distance <= max_distance");
    }

    if (j.contains("update_rate"))
    {
        const auto rate = j.at("update_rate").get<real_t>();
        if (rate <= 0.0)
        {
            throw std::logic_error("A range sensor needs a positive update_rate");
        }
        set_sampling_period(1.0 / rate);
    }
}

void RangeSensorBackendBase::load_from_json(const std::string &filename)
{
    std::ifstream file(filename);
    if (!file)
    {
        throw std::runtime_error("Could not open the sensor file " + filename);
    }
    load(nlohmann::json::parse(file));
}

real_t RangeSensorBackendBase::clamp_distance(real_t distance) const noexcept
{
    return std::clamp(distance, min_distance(), max_distance());
}

#ifdef BITSIM_CHRONO
void RangeSensorBackendBase::add_visual_shape(
    const std::unordered_map<std::string, std::any> &shape_properties)
{
    auto x_length = utils::resolve<float_t>("x_length", shape_properties);
    auto y_length = utils::resolve<float_t>("y_length", shape_properties);
    auto z_length = utils::resolve<float_t>("z_length", shape_properties);
    auto color = utils::resolve<chrono::ChColor>("chrono::ChColor", shape_properties);
    auto sensor_frame = utils::resolve<chrono::ChFrame<>>("chrono::ChFrame<>", shape_properties);

    auto sensor_vis =
        chrono_types::make_shared<chrono::ChVisualShapeBox>(x_length, y_length, z_length);
    sensor_vis->SetColor(color);

    // Attach to the body the sensor sits on
    this->body_->AddVisualShape(sensor_vis, sensor_frame);
}
#endif

} // namespace sensors::backends
} // namespace bitsim

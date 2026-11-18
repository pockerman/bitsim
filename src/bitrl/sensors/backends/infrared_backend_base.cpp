#include "bitrl/sensors/backends/infrared_backend_base.h"
#include "bitrl/bitrl_config.h"
#include "bitrl/sensors/backends/chrono_infrared_backend.h"
#include "bitrl/sensors/backends/mqtt_infrared_backend.h"
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

std::shared_ptr<InfraredBackendBase> InfraredBackendBase::create(const std::string &backend_type)
{
    if (backend_type == "SIMULATION" || backend_type == "CHRONO")
    {
#ifdef BITRL_CHRONO
        return std::make_shared<CHRONO_InfraredBackend>();
#else
        throw std::runtime_error("The " + backend_type +
                                 " infrared sensor backend needs bitrl built with Chrono");
#endif
    }

    if (backend_type == "MQTT")
    {
#ifdef BITRL_MQTT
        return std::make_shared<MQTT_InfraredBackend>();
#else
        throw std::runtime_error("The MQTT infrared sensor backend needs bitrl built with MQTT");
#endif
    }

    throw std::runtime_error("Unknown infrared sensor backend type: " + backend_type);
}

void InfraredBackendBase::set_elements(const std::vector<Position> &elements)
{
    if (elements.empty())
    {
        throw std::logic_error("An infrared sensor needs at least one element");
    }
    elements_ = elements;
    set_n_read_values(elements_.size());
}

void InfraredBackendBase::load(const nlohmann::json &j)
{
    if (j.contains("elements"))
    {
        std::vector<Position> elements;
        for (const auto &e : j.at("elements"))
        {
            elements.push_back(
                {e.at(0).get<real_t>(), e.at(1).get<real_t>(), e.at(2).get<real_t>()});
        }
        set_elements(elements);
    }

    if (j.contains("update_rate"))
    {
        const auto rate = j.at("update_rate").get<real_t>();
        if (rate <= 0.0)
        {
            throw std::logic_error("An infrared sensor needs a positive update_rate");
        }
        set_sampling_period(1.0 / rate);
    }
}

void InfraredBackendBase::load_from_json(const std::string &filename)
{
    std::ifstream file(filename);
    if (!file)
    {
        throw std::runtime_error("Could not open the sensor file " + filename);
    }
    load(nlohmann::json::parse(file));
}

#ifdef BITRL_CHRONO
void InfraredBackendBase::add_visual_shape(
    const std::unordered_map<std::string, std::any> &shape_properties)
{
    if (!body_)
    {
        throw std::logic_error("The infrared sensor has no body: call set_chrono_body()");
    }

    const auto x_length = utils::resolve<float_t>("x_length", shape_properties);
    const auto y_length = utils::resolve<float_t>("y_length", shape_properties);
    const auto z_length = utils::resolve<float_t>("z_length", shape_properties);
    const auto color = utils::resolve<chrono::ChColor>("chrono::ChColor", shape_properties);

    for (const auto &e : elements_)
    {
        auto shape =
            chrono_types::make_shared<chrono::ChVisualShapeBox>(x_length, y_length, z_length);
        shape->SetColor(color);
        body_->AddVisualShape(shape, chrono::ChFrame<>(chrono::ChVector3d(e[0], e[1], e[2])));
    }
}
#endif

} // namespace sensors::backends
} // namespace bitrl

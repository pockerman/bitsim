#include "bitrl/sensors/infrared_sensor.h"

#include <stdexcept>

namespace bitrl
{
namespace sensors
{

const std::string InfraredSensor::SENSOR_TYPE = "InfraredSensor";

std::shared_ptr<InfraredSensor> InfraredSensor::create(const std::string &backend_type,
                                                       const std::string &name)
{
    return std::make_shared<InfraredSensor>(backends::InfraredBackendBase::create(backend_type),
                                            name);
}

InfraredSensor::InfraredSensor(std::shared_ptr<backends::InfraredBackendBase> backend,
                               const std::string &name)
    : SensorBase(SENSOR_TYPE, name), backend_(backend)
{
    if (!backend_)
    {
        throw std::logic_error("InfraredSensor needs a backend");
    }
}

void InfraredSensor::init()
{
    values_.assign(backend_->n_elements(), 1.0);
    enable();
}

const std::vector<real_t> &InfraredSensor::read_values()
{
    if (!is_enabled())
    {
        return values_;
    }

    auto values = backend_->read_values();
    if (values.size() != backend_->n_elements())
    {
        throw std::logic_error("The infrared backend read " + std::to_string(values.size()) +
                               " values for " + std::to_string(backend_->n_elements()) +
                               " elements");
    }
    values_ = std::move(values);
    return values_;
}

void InfraredSensor::set_threshold(real_t threshold)
{
    if (threshold <= 0.0 || threshold >= 1.0)
    {
        throw std::logic_error("The threshold of an InfraredSensor must be in (0, 1)");
    }
    threshold_ = threshold;
}

bool InfraredSensor::is_dark(uint_t i) const { return values_.at(i) < threshold_; }

std::string InfraredSensor::pattern() const
{
    std::string pattern;
    for (uint_t i = 0; i < n_elements(); ++i)
    {
        pattern += is_dark(i) ? '#' : '.';
    }
    return pattern;
}

} // namespace sensors
} // namespace bitrl

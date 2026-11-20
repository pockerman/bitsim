#include "bitsim/sensors/imu_sensor.h"

#include <stdexcept>

namespace bitsim
{
namespace sensors
{

const std::string IMUSensor::SENSOR_TYPE = "IMUSensor";

std::shared_ptr<IMUSensor> IMUSensor::create(const std::string &backend_type,
                                             const std::string &name)
{
    return std::make_shared<IMUSensor>(backends::IMUBackendBase::create(backend_type), name);
}

IMUSensor::IMUSensor(std::shared_ptr<backends::IMUBackendBase> backend, const std::string &name)
    : SensorBase(SENSOR_TYPE, name), backend_(backend)
{
    if (!backend_)
    {
        throw std::logic_error("IMUSensor needs a backend");
    }
}

void IMUSensor::init()
{
    values_.assign(6, 0.0);
    enable();
}

const std::vector<real_t> &IMUSensor::read_values()
{
    if (!is_enabled())
    {
        return values_;
    }

    auto values = backend_->read_values();
    if (values.size() != 6)
    {
        throw std::logic_error("The IMU backend read " + std::to_string(values.size()) +
                               " values, not 6");
    }
    values_ = std::move(values);
    return values_;
}

std::array<real_t, 3> IMUSensor::acceleration() const
{
    return {values_.at(0), values_.at(1), values_.at(2)};
}

std::array<real_t, 3> IMUSensor::angular_velocity() const
{
    return {values_.at(3), values_.at(4), values_.at(5)};
}

} // namespace sensors
} // namespace bitsim

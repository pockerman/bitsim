#include "bitsim/actuators/dc_motor.h"

#include <algorithm>
#include <stdexcept>

namespace bitsim
{
namespace actuators
{

std::shared_ptr<DCMotor> DCMotor::create_simulated(const backends::DCMotorParameters &parameters,
                                                   const std::string &name)
{
    auto backend = std::make_shared<backends::SIM_DCMotorBackend>(parameters);
    return std::make_shared<DCMotor>(backend, name);
}

DCMotor::DCMotor(std::shared_ptr<backends::DCMotorBackendBase> backend, const std::string &name)
    : backend_(backend), name_(name)
{
    if (!backend_)
    {
        throw std::logic_error("DCMotor needs a backend");
    }
}

void DCMotor::set_speed(real_t speed)
{
    target_speed_ = std::clamp(speed, -max_speed_, max_speed_);
    backend_->set_target_speed(target_speed_);
}

void DCMotor::set_max_speed(real_t max_speed)
{
    if (max_speed <= 0.0)
    {
        throw std::logic_error("DCMotor: the maximum speed must be positive");
    }
    max_speed_ = max_speed;
    set_speed(target_speed_);
}

} // namespace actuators
} // namespace bitsim

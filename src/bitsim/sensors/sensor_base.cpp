#include "bitsim/sensors/sensor_base.h"

#include <string>

namespace bitsim
{
namespace sensors
{

std::string SensorBase::backend_type_str() const { return bitsim::consts::INVALID_STR; }
std::string SensorBase::sensor_units() const { return bitsim::consts::INVALID_STR; }
} // namespace sensors
} // namespace bitsim
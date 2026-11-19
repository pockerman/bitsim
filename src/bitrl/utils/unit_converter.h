#ifndef UNIT_CONVERTER_H
#define UNIT_CONVERTER_H

///
/// \file unit_converter.h
///
/// Various unit conversion utilities
///

#include "bitrl/bitrl_consts.h"
#include "bitrl/bitrl_types.h"

namespace bitrl
{
namespace utils::unit_converter
{

///
/// \fn rpm_to_angular_speed
/// \brief Convert RPM to angular speed in rad/s
///
inline real_t rpm_to_angular_speed(uint_t rpm)
{
    return (2.0 * bitrl::consts::maths::PI * rpm) / 60.0;
}

///
/// \fn angular_speed_to_linear_speed
/// \brief Convert angular speed to linear speed
///
inline real_t angular_speed_to_linear_speed(real_t r, real_t w) { return r * w; }

///
/// \fn rad_to_degrees
/// \brief Convert radians to degrees
///
inline real_t rad_to_degrees(real_t rad) { return (rad * 180.0) / bitrl::consts::maths::PI; }

///
/// \fn degrees_to_rad
/// \brief Convert degrees to radians
///
inline real_t degrees_to_rad(real_t degs) { return (degs * bitrl::consts::maths::PI) / 180.0; }

} // namespace utils::unit_converter
} // namespace bitrl

#endif
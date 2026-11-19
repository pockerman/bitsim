#ifndef GYMFCPP_CONSTS_H
#define GYMFCPP_CONSTS_H
/**
 * \file bitrl_consts.h
 *
 */

#include "bitrl/bitrl_config.h"
#include "bitrl/bitrl_types.h"
#include <filesystem>
#include <numbers>
#include <string>

namespace bitrl
{
namespace consts
{

/**
 * @brief Invalid id.
 */
inline const uint_t INVALID_ID = static_cast<uint_t>(-1);

/**
 * @brief Invalid string
 */
inline const std::string INVALID_STR = std::string("INVALID");

/**
 * @brief Tolerance used around the library
 */
inline const real_t TOLERANCE = 1.0e-8;

///
/// @brief The directory that holds the robot, sensor and world definitions
///
/// It is, in this order of preference:
/// - the directory in the environment variable BITRL_DATA_DIR, if it is set,
/// - the bitrl source tree the library was built from, if it still exists,
///   so that a build uses the files you edit,
/// - the directory the files are installed in, share/bitrl under the install prefix.
///
std::filesystem::path data_dir();

///
/// @brief Path to robots definitions
///
inline const std::string ROBOTS_DIR = (data_dir() / "robots").string();

///
/// @brief Path to sensors definitions
///
inline const std::string SENSORS_DIR = (data_dir() / "sensors").string();

/// Retruns an std::filesystem::path for the robot filename
/// @param filename The filename of the robot
/// @return An std::filesystem::path
inline std::filesystem::path build_robot_file_path(const std::string filename)
{
    return std::filesystem::path(ROBOTS_DIR) / filename;
}

namespace maths
{

///
/// @brief The PI constant
///
inline const real_t PI = std::numbers::pi;

/**
 * @brief Acceleration due to gravity m/secs
 */
inline const real_t G = 9.81;
} // namespace maths

} // namespace consts

} // namespace bitrl

#endif // GYMFCPP_CONSTS_H

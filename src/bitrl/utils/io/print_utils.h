#ifndef BITRL_UTILS_IO_PRINT_UTILS_H
#define BITRL_UTILS_IO_PRINT_UTILS_H

/**
 * \file print_utils.h
 * \brief Print labelled values, one per line, as the examples do: two spaces, the label padded
 * to a width, and the value. Numbers smaller than 5e-4 print as 0, so that round-off does not
 * show as -0.000. The overloads for Chrono vectors, matrices and frames need bitrl built with
 * Chrono.
 */

#include "bitrl/bitrl_config.h"
#include "bitrl/bitrl_types.h"

#include <ostream>
#include <string>

#ifdef BITRL_CHRONO
#include <chrono/core/ChFrame.h>
#include <chrono/core/ChMatrix33.h>
#include <chrono/core/ChVector3.h>
#endif

namespace bitrl
{
namespace utils
{
namespace io
{

///
/// @brief x, or 0 when \f$|x| <\f$ zero
///
real_t clean(real_t x, real_t zero = 5.0e-4);

///
/// @brief An empty line and then the title
///
void print_title(const std::string &title, std::ostream &out);
void print_title(const std::string &title);

///
/// @brief The label and the value, with 3 decimals
///
void print_number(const std::string &label, real_t value, int width = 26);

///
/// @brief The label and true or false
///
void print_flag(const std::string &label, bool value, int width = 26);

///
/// @brief The label and the value in scientific notation with one decimal, for small errors
///
void print_error(const std::string &label, real_t value, int width = 26);

///
/// @brief The label, the value right-aligned with the given decimals, and the unit
///
void print_value(const std::string &label, real_t value, const std::string &unit, int precision = 3,
                 int width = 30);

#ifdef BITRL_CHRONO

///
/// @brief "(x, y, z)" with 3 decimals
///
std::string to_string(const chrono::ChVector3d &v);

///
/// @brief The label and the vector as "(x, y, z)"
///
void print_vector(const std::string &label, const chrono::ChVector3d &v, int width = 26);

///
/// @brief The label on a line of its own, then the three rows of the matrix
///
void print_matrix(const std::string &label, const chrono::ChMatrix33d &R);

///
/// @brief The label, the origin of the frame and its three axes, as seen from its parent: the
/// columns of its rotation matrix
///
void print_frame(const std::string &label, const chrono::ChFrame<> &frame, int width = 14);

#endif // BITRL_CHRONO

} // namespace io
} // namespace utils
} // namespace bitrl

#endif // BITRL_UTILS_IO_PRINT_UTILS_H

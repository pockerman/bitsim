#include "bitrl/utils/io/print_utils.h"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace bitrl
{
namespace utils
{
namespace io
{

real_t clean(real_t x, real_t zero) { return std::abs(x) < zero ? 0.0 : x; }

void print_title(const std::string &title, std::ostream &out) { out << "\n" << title << "\n"; }

void print_title(const std::string &title) { print_title(title, std::cout); }

void print_number(const std::string &label, real_t value, int width)
{
    std::cout << "  " << std::left << std::setw(width) << label << " " << std::fixed
              << std::setprecision(3) << value << "\n";
}

void print_flag(const std::string &label, bool value, int width)
{
    std::cout << "  " << std::left << std::setw(width) << label << " " << std::boolalpha << value
              << "\n";
}

void print_error(const std::string &label, real_t value, int width)
{
    std::cout << "  " << std::left << std::setw(width) << label << " " << std::scientific
              << std::setprecision(1) << value << "\n";
}

void print_value(const std::string &label, real_t value, const std::string &unit, int precision,
                 int width)
{
    std::cout << "  " << std::left << std::setw(width) << label << " " << std::right << std::fixed
              << std::setprecision(precision) << std::setw(10) << value << " " << unit << "\n";
}

#ifdef BITRL_CHRONO

std::string to_string(const chrono::ChVector3d &v)
{
    std::ostringstream out;
    out << std::fixed << std::setprecision(3) << "(" << clean(v.x()) << ", " << clean(v.y()) << ", "
        << clean(v.z()) << ")";
    return out.str();
}

void print_vector(const std::string &label, const chrono::ChVector3d &v, int width)
{
    std::cout << "  " << std::left << std::setw(width) << label << " " << to_string(v) << "\n";
}

void print_matrix(const std::string &label, const chrono::ChMatrix33d &R)
{
    std::cout << "  " << label << "\n";
    for (int row = 0; row < 3; ++row)
    {
        std::cout << "    " << std::fixed << std::setprecision(3);
        for (int col = 0; col < 3; ++col)
        {
            std::cout << std::right << std::setw(7) << clean(R(row, col)) << " ";
        }
        std::cout << "\n";
    }
}

void print_frame(const std::string &label, const chrono::ChFrame<> &frame, int width)
{
    const auto &R = frame.GetRotMat();
    std::cout << "  " << std::left << std::setw(width) << label
              << " origin=" << to_string(frame.GetPos()) << "  x=" << to_string(R.GetAxisX())
              << "  y=" << to_string(R.GetAxisY()) << "  z=" << to_string(R.GetAxisZ()) << "\n";
}

#endif // BITRL_CHRONO

} // namespace io
} // namespace utils
} // namespace bitrl

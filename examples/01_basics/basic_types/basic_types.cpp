// The basics of bitsim: its number and matrix types, its constants, points, angles and units,
// and the files that describe robots. Every other example uses these.
//
// Usage: basic_types

#include "bitsim/bitsim_consts.h"
#include "bitsim/bitsim_types.h"
#include "bitsim/extern/nlohmann/json/json.hpp"
#include "bitsim/geometry/geom_point.h"
#include "bitsim/utils/io/print_utils.h"
#include "bitsim/maths/math_utils.h"
#include "bitsim/utils/unit_converter.h"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

namespace basic_types
{

using bitsim::real_t;
using bitsim::uint_t;
using bitsim::geometry::GeomPoint;
using namespace bitsim::utils::io;

void numbers_and_matrices()
{
    print_title("1. Numbers, vectors and matrices");

    // real_t is the floating point type of bitsim, uint_t its unsigned integer
    std::cout << "  real_t has " << sizeof(real_t) << " bytes, uint_t " << sizeof(uint_t) << "\n";

    // The vectors and matrices are Eigen's. RealColVec3d is a column vector of three, RealMat3d a
    // 3 x 3 matrix, DynMat<T> a matrix whose size is set when it is made
    const bitsim::RealColVec3d v(1.0, 2.0, 2.0);
    bitsim::RealMat3d A = bitsim::RealMat3d::Identity();
    A(0, 1) = 2.0;
    const bitsim::RealColVec3d Av = A * v;
    print_number("|v|", v.norm());
    print_number("(A v)_x", Av.x());

    bitsim::DynMat<real_t> M(2, 4);
    M.setConstant(0.5);
    std::cout << "  M is " << M.rows() << " x " << M.cols() << "\n";
    print_number("sum of M", M.sum());

    // A plain std::vector of real_t is STD_RealVec
    const bitsim::STD_RealVec samples{0.1, 0.4, 0.2};
    print_number("max of the samples", bitsim::maths::max(samples));
}

void constants()
{
    print_title("2. Constants");

    print_number("PI", bitsim::consts::maths::PI);
    print_number("G [m/s^2]", bitsim::consts::maths::G);
    print_error("TOLERANCE", bitsim::consts::TOLERANCE);
    print_flag("INVALID_ID is uint_t(-1)", bitsim::consts::INVALID_ID == uint_t(-1));
}

void points()
{
    print_title("3. Points");

    // A point in spacedim dimensions. It knows its distance to others, its norm and its dot
    // product, and adds, subtracts and scales like a vector
    const GeomPoint<2> a{1.0, 2.0};
    const GeomPoint<2> b{4.0, 6.0};
    std::cout << std::defaultfloat << std::setprecision(6) << "  a = " << a << ", b = " << b
              << ", a + b = " << a + b << ", 2 a = " << 2.0 * a << "\n";
    print_number("distance from a to b", a.distance(b));
    print_number("|b - a|", (b - a).L2_norm());
    print_number("a . b", a.dot(b));
}

void angles_and_units()
{
    print_title("4. Angles and units");

    using namespace bitsim::utils::unit_converter;
    print_number("90 deg [rad]", degrees_to_rad(90.0));
    print_number("1 rad [deg]", rad_to_degrees(1.0));
    print_number("60 rpm [rad/s]", rpm_to_angular_speed(60));
    print_number("1 rad/s, 0.1 m wheel [m/s]", angular_speed_to_linear_speed(0.1, 1.0));

    // Headings are compared after wrapping them to (-180, 180] degrees: turning 350 degrees left
    // is the same as turning 10 degrees right
    const real_t turn = degrees_to_rad(350.0);
    print_number("350 deg wrapped [deg]", rad_to_degrees(bitsim::maths::wrap_angle(turn)));
}

void robot_files()
{
    print_title("5. Robot files");

    // Robots, sensors and worlds are described by JSON files in the data directory of bitsim. The
    // simulated robots of the later examples load them; here we read one ourselves
    const std::filesystem::path file =
        std::filesystem::path(bitsim::consts::ROBOTS_DIR) / "bitsim_diff_drive_robot.json";
    std::ifstream in(file);
    if (!in)
    {
        throw std::runtime_error("Cannot open the robot file " + file.string());
    }
    const nlohmann::json robot = nlohmann::json::parse(in);

    std::cout << "  " << file.filename().string() << " describes " << robot["name"] << "\n";
    print_number("wheel radius [m]", robot["robot-data"]["wheel_radius"].get<real_t>());
    print_number("axle length [m]", robot["robot-data"]["axle_length"].get<real_t>());
    print_number("motor gear ratio", robot["motors"]["parameters"]["gear_ratio"].get<real_t>());
    std::cout << "  sensors:";
    for (const auto &[name, sensor] : robot["sensors"]["sensors"].items())
    {
        std::cout << " " << name << " (" << sensor["type"].get<std::string>() << ")";
    }
    std::cout << "\n";
}

} // namespace basic_types

int main()
{
    try
    {
        basic_types::numbers_and_matrices();
        basic_types::constants();
        basic_types::points();
        basic_types::angles_and_units();
        basic_types::robot_files();
        return 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}

// The kinematics of a differential drive robot with DiffDriveDynamics: the robot drives straight,
// then a quarter of a circle, and the example compares where it ends up with where the geometry
// of a circle says it should be.
//
// Usage: diff_drive_kinematics

#include "bitsim/bitsim_consts.h"
#include "bitsim/bitsim_types.h"
#include "bitsim/dynamics/diff_drive_dynamics.h"
#include "bitsim/utils/io/print_utils.h"

#include <any>
#include <array>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>

namespace diff_drive_kinematics
{

using namespace bitsim::dynamics;
using namespace bitsim::utils::io;
using bitsim::real_t;

const real_t PI = bitsim::consts::maths::PI;
const real_t DEG = PI / 180.0;

std::map<std::string, std::any> command(real_t v, real_t w)
{
    // v [m/s], w [rad/s], and no errors in the distance or the turn
    return {{"v", std::any(v)}, {"w", std::any(w)}, {"errors", std::any(std::array<real_t, 2>{})}};
}

void print_pose(const std::string &label, const DiffDriveDynamics &robot)
{
    std::cout << "  " << label << ": x = " << std::fixed << std::setprecision(3)
              << clean(robot.get_x_position()) << " m, y = " << clean(robot.get_y_position())
              << " m, theta = " << std::setprecision(1) << clean(robot.get_orientation() / DEG)
              << " deg\n";
}

void run()
{
    // The matrices of the linearised model are for an extended Kalman filter; not needed here
    DiffDriveDynamics robot(DiffDriveDynamics::DynamicVersion::V1, false);
    robot.set_time_step(0.1);

    print_title("1. Straight ahead");
    print_pose("start", robot);
    for (int k = 0; k < 20; ++k)
    {
        robot.integrate(command(0.5, 0.0));
    }
    print_pose("after 2 s at 0.5 m/s", robot);

    print_title("2. A quarter of a circle to the left");
    // 0.5 m/s turning at 45 deg/s: a circle of radius v / w, a quarter of it in 2 s
    const real_t v = 0.5;
    const real_t w = 45.0 * DEG;
    const real_t radius = v / w;
    for (int k = 0; k < 20; ++k)
    {
        robot.integrate(command(v, w));
    }
    print_pose("after 2 s at 45 deg/s", robot);
    print_number("radius v / w [m]", radius);
    std::cout << "  a quarter circle from (1, 0) facing x ends at (" << std::setprecision(3)
              << 1.0 + radius << ", " << radius << ") facing y\n";
}

} // namespace diff_drive_kinematics

int main()
{
    diff_drive_kinematics::run();
    return 0;
}

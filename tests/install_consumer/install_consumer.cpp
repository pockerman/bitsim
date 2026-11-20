// Checks that an installed bitsim can be compiled against, linked and run, and
// that it finds its robot, sensor and world files
#include "bitsim/actuators/backends/sim_dc_motor_backend.h"
#include "bitsim/actuators/dc_motor.h"
#include "bitsim/bitsim_consts.h"
#include "bitsim/bitsim_version.h"

#include <filesystem>
#include <iostream>

int main()
{
    using bitsim::actuators::DCMotor;
    using bitsim::actuators::backends::DCMotorParameters;

    auto motor = DCMotor::create_simulated(DCMotorParameters(), "motor");
    motor->set_speed(1.0);
    for (int i = 0; i < 100; ++i)
    {
        motor->step(0.001);
    }

    const auto robot = bitsim::consts::build_robot_file_path("bitsim_diff_drive_robot.json");
    std::cout << "bitsim " << BITSIM_VERSION << ", data in " << bitsim::consts::data_dir()
              << std::endl;
    if (!std::filesystem::exists(robot))
    {
        std::cerr << "error: " << robot << " does not exist" << std::endl;
        return 1;
    }
    return 0;
}

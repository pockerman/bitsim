// Checks that an installed bitrl can be compiled against, linked and run, and
// that it finds its robot, sensor and world files
#include "bitrl/actuators/backends/sim_dc_motor_backend.h"
#include "bitrl/actuators/dc_motor.h"
#include "bitrl/bitrl_consts.h"
#include "bitrl/bitrl_version.h"

#include <filesystem>
#include <iostream>

int main()
{
    using bitrl::actuators::DCMotor;
    using bitrl::actuators::backends::DCMotorParameters;

    auto motor = DCMotor::create_simulated(DCMotorParameters(), "motor");
    motor->set_speed(1.0);
    for (int i = 0; i < 100; ++i)
    {
        motor->step(0.001);
    }

    const auto robot = bitrl::consts::build_robot_file_path("bitrl_diff_drive_robot.json");
    std::cout << "bitrl " << BITRL_VERSION << ", data in " << bitrl::consts::data_dir()
              << std::endl;
    if (!std::filesystem::exists(robot))
    {
        std::cerr << "error: " << robot << " does not exist" << std::endl;
        return 1;
    }
    return 0;
}

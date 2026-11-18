#include "bitrl/bitrl_config.h"

#ifdef BITRL_CHRONO

#include "bitrl/sensors/backends/chrono_imu_backend.h"

#include <chrono/physics/ChSystem.h>

#include <stdexcept>

namespace bitrl
{
namespace sensors::backends
{

const std::string CHRONO_IMUBackend::BACKEND_TYPE = "SIMULATION";

void CHRONO_IMUBackend::Errors::read(const nlohmann::json &j)
{
    if (j.contains("gyro_bias"))
    {
        const auto &b = j.at("gyro_bias");
        gyro_bias = {b.at(0).get<real_t>(), b.at(1).get<real_t>(), b.at(2).get<real_t>()};
    }
    gyro_noise = j.value("gyro_noise", gyro_noise);
    accel_noise = j.value("accel_noise", accel_noise);
    seed = j.value("seed", seed);
}

CHRONO_IMUBackend::CHRONO_IMUBackend() : IMUBackendBase(BACKEND_TYPE) {}

CHRONO_IMUBackend::CHRONO_IMUBackend(std::shared_ptr<chrono::ChBody> body)
    : IMUBackendBase(BACKEND_TYPE)
{
    set_chrono_body(body);
}

void CHRONO_IMUBackend::set_errors(const Errors &errors)
{
    if (errors.gyro_noise < 0.0 || errors.accel_noise < 0.0)
    {
        throw std::logic_error("The noise of an IMU cannot be negative");
    }
    errors_ = errors;
    generator_.seed(errors_.seed);
}

void CHRONO_IMUBackend::load(const nlohmann::json &j)
{
    IMUBackendBase::load(j);
    Errors errors = errors_;
    errors.read(j);
    set_errors(errors);
}

std::vector<real_t> CHRONO_IMUBackend::read_values()
{
    if (!body_)
    {
        throw std::logic_error("CHRONO_IMUBackend has no body: call set_chrono_body()");
    }
    const auto *system = body_->GetSystem();
    if (system == nullptr)
    {
        throw std::logic_error("The body of the CHRONO_IMUBackend is in no system");
    }

    // The acceleration of the point where the IMU sits, from the motion of
    // the body: a_P = a + alpha x r + omega x (omega x r)
    const auto &frame = body_->GetFrameRefToAbs();
    const auto &p = position();
    const chrono::ChVector3d r =
        frame.TransformDirectionLocalToParent(chrono::ChVector3d(p[0], p[1], p[2]));
    const chrono::ChVector3d omega = body_->GetAngVelParent();
    const chrono::ChVector3d acceleration =
        body_->GetPosDt2() + body_->GetAngAccParent().Cross(r) + omega.Cross(omega.Cross(r));

    // The accelerometer feels every acceleration but gravity's
    const chrono::ChVector3d specific_force = frame.TransformDirectionParentToLocal(
        acceleration - system->GetGravitationalAcceleration());
    const chrono::ChVector3d rate = body_->GetAngVelLocal();

    std::normal_distribution<real_t> accel_noise(0.0, errors_.accel_noise);
    std::normal_distribution<real_t> gyro_noise(0.0, errors_.gyro_noise);
    std::vector<real_t> values(6);
    for (int i = 0; i < 3; ++i)
    {
        values[i] = specific_force[i] + (errors_.accel_noise > 0.0 ? accel_noise(generator_) : 0.0);
        values[3 + i] = rate[i] + errors_.gyro_bias[i] +
                        (errors_.gyro_noise > 0.0 ? gyro_noise(generator_) : 0.0);
    }
    return values;
}

} // namespace sensors::backends
} // namespace bitrl

#endif

// Sensor fusion over MQTT with ExtendedKalmanFilter. A Python program, extended_kalman_filter.py,
// simulates a robot that drives a circle and publishes its noisy GPS position and its noisy
// inputs. This program reads them from the broker, predicts with each input, corrects with each
// GPS reading, and publishes the estimate back. The filter is the one of the other examples; MQTT
// only carries the readings in and the estimate out.
//
// Usage: ekf_sensor_fusion_mqtt [BROKER]     (default tcp://localhost:1883)

#include "bitrl/bitrl_config.h"

#ifdef BITRL_MQTT

#include "bitrl/bitrl_consts.h"
#include "bitrl/bitrl_types.h"
#include "bitrl/dynamics/system_state.h"
#include "bitrl/estimation/extended_kalman_filter.h"
#include "bitrl/network/mqtt_publisher.h"
#include "bitrl/network/mqtt_subscriber.h"
#include "bitrl/sensors/messages/vector_message.h"
#include "bitrl/utils/unit_converter.h"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>

namespace ekf_sensor_fusion_mqtt
{
using bitrl::DynMat;
using bitrl::DynVec;
using bitrl::real_t;
using bitrl::uint_t;

const std::string GPS_TOPIC = "GPS_TOPIC";
const std::string INPUT_TOPIC = "U_TOPIC";
const std::string STATE_TOPIC = "STATE_TOPIC";
const real_t DT = 0.1; // [s] between two readings, as in the Python program

// Wait this long for the Python program to start, and stop when it has sent nothing for
// QUIET_TIME
const std::chrono::milliseconds START_TIME(60000);
const std::chrono::milliseconds QUIET_TIME(5000);

// The state is (x, y, yaw, v); the input u = (v, omega). Each step
//   x += v cos(yaw) dt,  y += v sin(yaw) dt,  yaw += omega dt,  v = u_v
// as in the Python program, whose Jacobian this one copies
class MotionModel
{
  public:
    using state_type = bitrl::dynamics::SysState<4>;
    using matrix_type = DynMat<real_t>;
    using input_type = DynVec<real_t>;

    MotionModel()
        : state_({"X", "Y", "YAW", "V"}, 0.0), F_(matrix_type::Identity(4, 4)),
          L_(matrix_type::Identity(4, 4))
    {
    }

    state_type &evaluate(const input_type &u)
    {
        const real_t yaw = state_[2];
        const real_t v = u[0];

        // The Jacobian at the state before the step
        F_.setIdentity();
        F_(0, 2) = -DT * v * std::sin(yaw);
        F_(0, 3) = DT * std::cos(yaw);
        F_(1, 2) = DT * v * std::cos(yaw);
        F_(1, 3) = DT * std::sin(yaw);

        state_[0] += DT * v * std::cos(yaw);
        state_[1] += DT * v * std::sin(yaw);
        state_[2] += DT * u[1];
        state_[3] = v;
        return state_;
    }

    // F, and L = I: the noise Q is added to the state as it is
    const matrix_type &get_matrix(const std::string &name) const
    {
        if (name == "F")
        {
            return F_;
        }
        if (name == "L")
        {
            return L_;
        }
        throw std::invalid_argument("MotionModel has no matrix " + name);
    }

    const state_type &get_state() const { return state_; }
    state_type &get_state() { return state_; }

  private:
    state_type state_;
    matrix_type F_;
    matrix_type L_;
};

// The GPS measures x and y
class GPSModel
{
  public:
    GPSModel() : H_(DynMat<real_t>::Zero(2, 4))
    {
        H_(0, 0) = 1.0;
        H_(1, 1) = 1.0;
    }

    DynVec<real_t> evaluate(const DynVec<real_t> &x) const
    {
        DynVec<real_t> z(2);
        z << x[0], x[1];
        return z;
    }

    const DynMat<real_t> &get_matrix(const std::string &name) const
    {
        if (name != "H")
        {
            throw std::invalid_argument("GPSModel has no matrix " + name);
        }
        return H_;
    }

  private:
    DynMat<real_t> H_;
};

std::optional<DynVec<real_t>> parse(const std::optional<std::string> &message)
{
    if (!message)
    {
        return std::nullopt;
    }
    const auto vector = bitrl::sensors::EigenVectorMessage<real_t>::parse(*message);
    if (!vector)
    {
        return std::nullopt;
    }
    return vector->message;
}

int run(const std::string &broker)
{
    using bitrl::utils::unit_converter::degrees_to_rad;

    MotionModel motion;
    const GPSModel gps;
    bitrl::estimation::ExtendedKalmanFilter<MotionModel> ekf(motion);

    // The matrices of the Python program
    DynMat<real_t> Q = DynMat<real_t>::Zero(4, 4);
    Q(0, 0) = 0.1 * 0.1;                                 // x [m^2]
    Q(1, 1) = 0.1 * 0.1;                                 // y [m^2]
    Q(2, 2) = degrees_to_rad(1.0) * degrees_to_rad(1.0); // yaw [rad^2]
    Q(3, 3) = 1.0;                                       // v [m^2/s^2]
    const DynMat<real_t> R = DynMat<real_t>::Identity(2, 2);
    ekf.with_matrix("P", DynMat<real_t>::Identity(4, 4)).with_matrix("Q", Q);

    bitrl::network::MqttSubscriber gps_topic(broker, GPS_TOPIC);
    bitrl::network::MqttSubscriber input_topic(broker, INPUT_TOPIC);
    bitrl::network::MqttPublisher state_topic(broker, STATE_TOPIC);
    gps_topic.connect();
    input_topic.connect();
    state_topic.connect();
    std::cout << "Listening on " << broker << " for " << GPS_TOPIC << " and " << INPUT_TOPIC
              << "; the estimate goes to " << STATE_TOPIC << "\n"
              << "Start extended_kalman_filter.py now\n\n"
              << "  step    gps x    gps y   est x [m]  est y [m]  yaw [deg]  v [m/s]\n";

    uint_t n_steps = 0;
    uint_t n_bad = 0;
    while (true)
    {
        // The Python program publishes a GPS reading, then the input of the same step
        const auto z = parse(gps_topic.poll(n_steps + n_bad == 0 ? START_TIME : QUIET_TIME));
        if (!z)
        {
            break;
        }
        const auto u = parse(input_topic.poll(std::chrono::milliseconds(1000)));
        if (!u || u->size() != 2 || z->size() != 2)
        {
            ++n_bad;
            continue;
        }

        ekf.predict(*u);
        ekf.update(gps, *z, R);
        ++n_steps;

        const auto &state = motion.get_state();
        state_topic.publish(state.as_json());
        std::cout << std::fixed << std::setprecision(3) << std::setw(6) << n_steps << std::setw(9)
                  << (*z)[0] << std::setw(9) << (*z)[1] << std::setw(12) << state[0]
                  << std::setw(11) << state[1] << std::setprecision(1) << std::setw(11)
                  << state[2] * 180.0 / bitrl::consts::maths::PI << std::setprecision(3)
                  << std::setw(9) << state[3] << std::endl;
    }

    std::cout << "\nNo GPS reading for " << QUIET_TIME.count() / 1000 << " s: " << n_steps
              << " steps, " << n_bad << " readings that could not be used" << std::endl;
    return n_steps > 0 ? 0 : 1;
}

} // namespace ekf_sensor_fusion_mqtt

int main(int argc, char **argv)
{
    const std::string broker = argc > 1 ? argv[1] : "tcp://localhost:1883";
    try
    {
        return ekf_sensor_fusion_mqtt::run(broker);
    }
    catch (const std::exception &e)
    {
        std::cerr << "ekf_sensor_fusion_mqtt: " << e.what() << std::endl;
        return 1;
    }
}

#else

#include <iostream>

int main()
{
    std::cerr << "This example requires MQTT to be enable. Reconfigure bitrl with ENABLE_MQTT=ON"
              << std::endl;
    return 1;
}

#endif

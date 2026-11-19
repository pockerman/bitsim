\page bitrl_example_ekf_sensor_fusion_mqtt Sensor fusion over MQTT with an extended Kalman filter

\ref bitrl_example_extended_kalman_filter ran the filter on data it made itself. Here the data
comes over MQTT, as it would from a robot: a Python program simulates a robot that drives a circle
and publishes its noisy GPS position and its noisy inputs, and the C++ program fuses them with
\ref bitrl::sensors::EKFSensorFusion "EKFSensorFusion" and publishes the estimate back.

## What you need

- bitrl built with MQTT; see \ref bitrl_install;
- an MQTT broker on `localhost:1883`, such as Mosquitto;
- Python with `paho-mqtt`, `numpy` and `matplotlib`, for `extended_kalman_filter.py`.

## The model

The state is \f$(x, y, \psi, v)\f$: the position, the heading and the speed. The input is the speed
and the rate of turn, \f$u = (v, \omega)\f$, and the motion over a step \f$\Delta t\f$ = 0.1 s is

\f[
x \mathrel{+}= v \cos\psi\, \Delta t, \qquad y \mathrel{+}= v \sin\psi\, \Delta t, \qquad
\psi \mathrel{+}= \omega\, \Delta t
\f]

`MotionModel` in the code implements it and its Jacobian; `ObservationModel` says that the GPS
measures \f$x\f$ and \f$y\f$. The covariances are those of the Python program: \f$Q\f$ for the
motion, \f$R\f$ = I for the GPS.

## The topics

| Topic | From | What |
| ----- | ---- | ---- |
| `GPS_TOPIC` | Python | the noisy position, `[x, y]` |
| `U_TOPIC` | Python | the noisy input, `[v, w]` |
| `STATE_TOPIC` | C++ | the estimated state, as JSON |

`EKFSensorFusion` subscribes to the input and sensor topics: each `step()` predicts with the last
input and corrects with the last GPS reading.

## Running the example

@code{.sh}
mosquitto                                  # the broker, if none is running
./ekf_sensor_fusion_mqtt                   # in one terminal
python extended_kalman_filter.py           # in another, from this example's directory
@endcode

The Python program also runs its own filter on the same data, to compare with.

## The code

\include ekf_sensor_fusion_mqtt.cpp

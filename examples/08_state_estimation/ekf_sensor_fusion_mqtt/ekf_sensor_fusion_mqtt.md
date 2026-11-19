\page bitrl_example_ekf_sensor_fusion_mqtt Sensor fusion over MQTT with an extended Kalman filter

\ref bitrl_example_extended_kalman_filter ran the filter on data it made itself. Here the data
comes over MQTT, as it would from a robot: a Python program simulates a robot that drives a circle
and publishes its noisy GPS position and its noisy inputs, and the C++ program fuses them with
\ref bitrl::estimation::ExtendedKalmanFilter "ExtendedKalmanFilter" and publishes the estimate back.
The filter is the same as in the other examples: MQTT only carries the readings in and the estimate
out.

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

and \f$v\f$ is the speed of the input. `MotionModel` in the code implements it, with its Jacobian
\f$F\f$ and \f$L = I\f$, so that the noise \f$Q\f$ adds to the state as it is; `GPSModel` says
that the GPS measures \f$x\f$ and \f$y\f$. The covariances are those of the Python program:
\f$Q\f$ for the motion, \f$R\f$ = I for the GPS.

## The topics

| Topic | From | What |
| ----- | ---- | ---- |
| `GPS_TOPIC` | Python | the noisy position, `[x, y]` |
| `U_TOPIC` | Python | the noisy input, `[v, w]` |
| `STATE_TOPIC` | C++ | the estimated state, as JSON |

The Python program publishes a GPS reading and then the input of the same step. For each GPS
reading the C++ program takes the input from its \ref bitrl::network::MqttSubscriber "MqttSubscriber",
predicts with it, corrects with the reading, and publishes the state with an
\ref bitrl::network::MqttPublisher "MqttPublisher":

@code{.cpp}
const auto z = parse(gps_topic.poll(timeout));
const auto u = parse(input_topic.poll(std::chrono::milliseconds(1000)));
ekf.predict(*u);
ekf.update(gps, *z, R);
state_topic.publish(motion.get_state().as_json());
@endcode

It waits up to a minute for the first reading, and stops when none has come for 5 s.

## Running the example

@code{.sh}
mosquitto                                  # the broker, if none is running
./ekf_sensor_fusion_mqtt                   # in one terminal; or ./ekf_sensor_fusion_mqtt tcp://host:1883
python extended_kalman_filter.py           # in another, from this example's directory
@endcode

The Python program drives for 50 s, in real time. It also runs its own filter on the same data and
prints its estimate next to the one it receives, which is a step behind.

## Output

@code
Listening on tcp://localhost:1883 for GPS_TOPIC and U_TOPIC; the estimate goes to STATE_TOPIC
Start extended_kalman_filter.py now

  step    gps x    gps y   est x [m]  est y [m]  yaw [deg]  v [m/s]
     1    0.344    0.188       0.174      0.095        0.4    0.026
     2    0.368    0.037       0.213      0.075       -0.9   -0.421
     3    0.318    0.249       0.359      0.120       -0.0    1.612
...
   499   -9.312    6.880      -9.545      7.265      282.4    0.812
   500   -9.584    7.064      -9.544      7.210      283.6   -0.078

No GPS reading for 5 s: 500 steps, 0 readings that could not be used
@endcode

The readings are random, so each run differs. Over the 500 steps the C++ filter and the Python one
agree to within the 1 mm the program prints. The GPS readings are scattered by about 0.5 m; the
estimate is smoother. The speed in the state is the input's, which is noisy, as the model takes
\f$v\f$ from the input and does not filter it.

## The code

\include ekf_sensor_fusion_mqtt.cpp

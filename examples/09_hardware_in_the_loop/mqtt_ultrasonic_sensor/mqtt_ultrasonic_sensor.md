\page bitsim_example_mqtt_ultrasonic_sensor A real ultrasonic sensor over MQTT

\ref bitsim_example_chrono_ultrasonic_sensor with the backend swapped: the readings come from a
sensor on a board that publishes them over MQTT, read by
\ref bitsim::sensors::backends::MQTT_UltrasonicBackend "MQTT_UltrasonicBackend". The program reads
the sensor the same way.

## What you need

- bitsim built with MQTT; see \ref bitsim_install;
- an MQTT broker on `localhost:1883`, such as Mosquitto;
- something that publishes readings on the topic `ultrasound`: a board with an HC-SR04, or
  `feed.py` in this example's directory, which needs Python and `paho-mqtt`.

## The messages

Each reading is a JSON object with the distance, its unit and when it was measured:

@code
{"distance": 0.42, "unit": "cm", "timestamp": "2026-11-19 10:00:00.000000+00:00"}
@endcode

`feed.py` publishes one every 2 s, with the distance growing by 0.01 each time. The example prints
the distance the sensor returns, and what the last message said.

## Running the example

@code{.sh}
mosquitto            # the broker, if none is running
./mqtt_ultrasonic_sensor
python feed.py       # in another terminal
@endcode

The robot of the line following examples reads its sonar through the same backend when its robot
file says `"backend": "MQTT"`; see \ref bitsim_sensors.

## The code

\include mqtt_ultrasonic_sensor.cpp

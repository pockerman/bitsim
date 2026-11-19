\page bitrl_example_chrono_ultrasonic_sensor A simulated ultrasonic sensor

The smallest use of a sensor in bitrl: an ultrasonic range sensor on a box, simulated with Chrono,
measuring the distance to another box in front of it. The line following examples use the same
sensor on a robot; see \ref bitrl_sensors for all the sensors.

## What you need

bitrl built with Chrono; see \ref bitrl_install.

## Sensors and backends

A sensor in bitrl is a \ref bitrl::sensors::SensorBase "SensorBase" with a *backend* that does the
measuring. \ref bitrl::sensors::UltrasonicSensor "UltrasonicSensor" reads a range backend:

- \ref bitrl::sensors::backends::CHRONO_UltrasonicBackend "CHRONO_UltrasonicBackend" casts rays
  from the body that carries it into the Chrono world and returns the distance to the nearest hit,
  up to its maximum range;
- \ref bitrl::sensors::backends::MQTT_UltrasonicBackend "MQTT_UltrasonicBackend" reads a real
  sensor on a board that publishes over MQTT, see \ref bitrl_example_mqtt_ultrasonic_sensor.

The program does not change with the backend: it calls `read_values()` on the sensor.

## The scene

A fixed 2 x 2 m box, the target, has its centre 2 m ahead of a fixed 0.3 m box that carries the
sensor. The face of the target is 1 m from the centre of the carrier, and the sensor reads 1 m at
every step.

## Running the example

@code{.sh}
./chrono_ultrasonic_sensor
@endcode

## Output

@code
Sensor name: MyUltrasonicSensor
Sensor type: UltrasonicSensor
Sensor backend: SIMULATION
Measured distance: 1 in: m
...
Measured distance: 1 in: m
@endcode

## The code

\include chrono_ultrasonic_sensor.cpp

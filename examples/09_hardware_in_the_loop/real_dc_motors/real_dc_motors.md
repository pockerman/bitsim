\page bitsim_example_real_dc_motors A differential drive robot with real motors

This example accompanies the sixth lesson of the robotics course. It brings together the differential
drive robot of \ref bitsim_example_chrono_diff_drive and the motor model of \ref bitsim_example_bldc_motor.
The two wheel motors are now real: two DC gear motors with encoders, driven by an Arduino. The robot they
drive is still simulated in Chrono. The simulation sends the board the speed each wheel should turn at,
and the board sends back the distance each wheel has covered. The simulation turns the wheels by that
much, and the robot moves.

You need _bitsim_ configured with Chrono and MQTT support, and an MQTT broker such as Mosquitto. The
hardware is optional: the example can simulate the motors as well, or run a fake board on the computer.

## Hardware in the loop

Testing a robot only in simulation can hide problems the hardware has, and testing it only on the
hardware is slow and risky. **Hardware in the loop** (HIL) is the middle ground: some parts are real and
the rest is simulated. Here the motors, their encoders and their speed controllers are real, and the
robot body, its wheels and the floor are simulated. The loop is:

1. the simulation decides how fast each wheel should turn, and sends it to the board
2. the board runs each motor at that speed, and sends back the distance its encoder has counted
3. the simulation turns each simulated wheel to that distance, divided by the wheel radius, and Chrono
   works out how the robot moves on the floor

@note Lift the real robot, or just the motors, so that the wheels turn freely. The real motors do not
carry the robot; the simulated ones do. We will see what that means for the result.

## The pieces

The example adds three things to _bitsim_.

**A DC motor.** `bitsim::actuators::DCMotor` is a motor run in speed control. You set the target speed of
its output shaft, step it, and read its angle and speed. What the motor is depends on its backend, in the
same way as the sensors of the robot in \ref bitsim_example_line_following_infrared_sonar, as follows.

- `SIM_DCMotorBackend` is a model of a DC gear motor, its encoder and its speed controller. It runs in the
  simulation only.
- `MQTT_DCMotorBackend` is a real motor on a board. The target goes to the board, and the angle comes back.

**A board.** One board drives both motors over one connection, so the motors share it.
`bitsim::boards::DCMotorBoardBase` is a board that drives DC motors: it takes a target speed per motor and
reports what its encoders measure. `bitsim::boards::arduino::DCArduinoBoardImpl` is an Arduino that does
it over MQTT.

**A robot that uses them.** `CHRONO_DiffDriveRobot` now has a `DCMotor` on each wheel. The motors come from
the new `motors` section of `robots/bitsim_diff_drive_robot.json`, and `set_motor` replaces them. Before
every Chrono step, `update` steps the motors and passes their output to the wheels:

@code{.cpp}
auto board = std::make_shared<DCArduinoBoardImpl>(config);
board->connect();

CHRONO_DiffDriveRobot robot;
robot.load(robot_file);
for (uint_t m = 0; m < 2; ++m)
{
    auto backend = std::make_shared<MQTT_DCMotorBackend>(board, m, OBSERVER_BANDWIDTH);
    robot.set_motor(m, std::make_shared<DCMotor>(backend));
}
robot.add_to_sys(system);

while (running)
{
    robot.set_velocity(v, omega);
    robot.update(STEP_SIZE);
    system.DoStepDynamics(STEP_SIZE);
}
@endcode

`set_velocity` turns a forward speed \f$v\f$ and a turning rate \f$\omega\f$ into the wheel speeds of
the differential drive, \f$(v \mp \omega L / 2) / r\f$ for the left and the right wheel, with \f$L\f$ the
distance between the wheels and \f$r\f$ their radius.

## Two ways to turn a wheel

A simulated motor and a real one connect to Chrono in different ways.

A **simulated motor** produces a torque. Its electrical part is the one of bldc_motor,

\f[
L \frac{di}{dt} = V - R i - K_e N \omega,
\f]

with \f$\omega\f$ the speed of the wheel and \f$N\f$ the gear ratio. It puts the torque \f$N K_e i\f$ on
the wheel, through a `ChLinkMotorRotationTorque`. Chrono moves the wheel, and the motor reads back how
fast it turns, so the motor feels the weight of the robot.

A **real motor** has already turned by the time we hear from it. So the wheel is connected through a
`ChLinkMotorRotationAngle`, and Chrono turns it to the angle the board reported. Chrono then tells us the
torque that took.

The robot chooses the link from the backend, so the rest of the code does not need to know which motors
it has.

## The speed controller

The board runs a PI controller on each motor every 10 ms. It measures the speed from the change of the
encoder counts and filters it. A feedforward gives the duty cycle that balances the back-EMF at the
target speed, and the PI controller corrects the rest:

@code{.cpp}
const float feedforward = c.target * GEAR_RATIO * EMF_CONSTANT / SUPPLY_VOLTAGE;
const float error = c.target - c.measured;
const float unclamped = feedforward + SPEED_KP * error + c.integral;
const float duty = constrain(unclamped, -1.0, 1.0);
@endcode

`SIM_DCMotorBackend` runs the same controller with the same gains. The constants are those of a 12 V
JGA25-370 gear motor with a 30:1 gearbox, about 300 rpm at the output. They are in the robot file:

| parameter | value | |
|---|---|---|
| `resistance` | 7 | Ohm |
| `inductance` | 0.002 | H |
| `emf_constant` | 0.0127 | V s/rad, at the rotor |
| `gear_ratio` | 30 | |
| `supply_voltage` | 12 | V |
| `speed_kp`, `speed_ki` | 0.03, 0.1 | duty per rad/s, duty per rad |
| `control_period` | 0.01 | s |
| `encoder_counts` | 660 | per wheel turn |

The gains are small on purpose. The same controller has to work on a free motor, which has almost no
inertia, and on a motor that pushes the robot, which has a lot of it. Larger gains make the free motor
oscillate.

## MQTT

The board and the simulation exchange JSON messages on two topics, 50 times a second:

| topic | from | message |
|---|---|---|
| `bitsim/dc_board/command` | simulation | `{"seq": 12, "left_speed": 2.5, "right_speed": 2.5}` |
| `bitsim/dc_board/state` | board | `{"ack": 12, "left_distance": 0.53, "right_distance": 0.52, "left_speed": 2.49, "right_speed": 2.51, "t_ms": 81234}` |

Speeds are wheel speeds in rad/s and distances are in m, counted from when the board started. The board
turns encoder counts into distance with `WHEEL_RADIUS`, which must be the wheel radius of the simulated
robot, 0.1 m. `ack` is the number of the last command the board received, which gives the round trip
time. If the commands stop for half a second, the board stops the motors.

The broker must accept connections from the network. With Mosquitto 2 that needs these two lines in its
configuration:

@code
listener 1883
allow_anonymous true
@endcode

Anonymous access is fine on a home network for a lesson. On a shared network, set up users and
passwords.

## The board

The sketch is in `real_dc_motors_board`. It is written for:

- an Arduino UNO R4 WiFi. A Nano 33 IoT should also work, but it has not been tested
- a TB6612FNG dual motor driver
- two DC gear motors with quadrature encoders, such as the JGA25-370

| | left motor | right motor |
|---|---|---|
| PWM | 5 | 6 |
| direction | 8, 9 | 10, 11 |
| encoder A (interrupt) | 2 | 3 |
| encoder B | 4 | 7 |

Standby of the driver is on pin 12. The right motor faces the other way on the robot, so the sketch
reverses it. The pins, the motor constants and the gains are at the top of the sketch. It needs the
**ArduinoMqttClient** library from the library manager, and has been compiled for the UNO R4 WiFi but not
yet run on real hardware.

Copy `arduino_secrets.example.h` to `arduino_secrets.h` and fill in your WiFi name, password and the
address of the computer with the broker. That file is ignored by git, so the password stays on your
computer.

## Following the board smoothly

The board reports every 20 ms, while Chrono steps every 10 ms. In between, the MQTT backend predicts the
angle from the last report and the speed. A wheel that follows an angle is differentiated twice by Chrono,
so every small correction of the prediction, and the noise of the encoder counts, would come out as a
spike of torque. So the wheel follows a smooth estimate instead, a second-order filter pulled towards
the predicted angle and the reported speed:

@code{.cpp}
const real_t acceleration = w * w * (predicted_angle_() - estimate_) +
                            2.0 * w * (reported_speed_ - estimate_rate_);
estimate_rate_ += dt * acceleration;
estimate_ += dt * estimate_rate_;
@endcode

The bandwidth \f$w\f$ is 10 rad/s. Faster, and more of the encoder noise reaches Chrono. Slower, and the
wheels lag behind the motors.

## Running the example

@code
./real_dc_motors --simulated-motors [--render]     # no hardware and no broker
./real_dc_motors --fake-board [broker]             # in one terminal, or connect the real board
./real_dc_motors [--render] [broker]               # in another
@endcode

The broker defaults to `tcp://localhost:1883`. The robot drives a square with sides of 1 m: forward at
0.25 m/s for 4 s, stop for 1 s, turn left on the spot at 45 deg/s for 2 s, stop for 1 s, four times.
Each second it prints the targets and speeds of the wheels, the distances they covered, the pose of the
robot as Chrono has it, and the pose from **odometry**: where the robot thinks it is, from the wheel
distances alone.

`--fake-board` stands in for the Arduino. It uses the same topics and messages, and two
`SIM_DCMotorBackend` motors that turn freely, as the real ones do with the wheels off the ground.

With `--render`, the robot drives the square over and over in an Irrlicht window. The white line is the
plan, the yellow line is where the robot went, and the red line is where odometry thinks it went.

### Simulated motors

@code
2. Driving a square with sides of 1 m
                     target       speed      distance      robot (Chrono)      odometry
  t (s)  segment   (rad/s)      (rad/s)       (m)        x     y    th      x     y    th
    1.0  forward     2.50  2.50   3.07  3.07   0.190  0.190    0.19  0.00     0    0.19  0.00     0
    2.0  forward     2.50  2.50   2.87  2.87   0.500  0.500    0.50  0.00     0    0.50  0.00     0
    3.0  forward     2.50  2.50   2.41  2.41   0.761  0.761    0.76  0.00     0    0.76  0.00     0
    4.0  forward     2.50  2.50   2.44  2.44   1.003  1.003    1.01  0.00    -0    1.00  0.00    -0
    5.0  stop        0.00  0.00  -0.55 -0.55   1.065  1.065    1.06  0.00     0    1.06  0.00     0
    6.0  turn left  -1.57  1.57  -2.14  1.77   0.894  1.131    1.01 -0.01    35    1.01 -0.01    34
    7.0  turn left  -1.57  1.57  -1.63  1.92   0.707  1.332    1.01  0.00    90    1.01  0.00    90
    8.0  stop        0.00  0.00   0.20 -0.57   0.655  1.374    1.02 -0.01   103    1.02 -0.01   103
    9.0  forward     2.50  2.50   3.23  2.77   0.869  1.508    1.00  0.17    91    1.00  0.17    91
    ...

3. Summary
  wheel distances left / right    2.742 / 5.258 m
                                          x       y   heading
  end of the square, planned          0.000   0.000     0.0 deg
  end of the square, Chrono           0.006  -0.012     0.4 deg
  end of the square, odometry         0.006  -0.012     0.5 deg
  odometry error                  0.000 m, 0.1 deg
  largest wheel torque            0.117 N m
  stall torque of a motor at 12 V 0.653 N m
@endcode

The motors carry the robot, so they are slow to change speed. They overshoot the target at the start of
each side, and keep turning after each stop: the first turn ends at 103 degrees instead of 90. The right
wheel then starts faster than the left, which turns the robot back towards 90 degrees. The square still
closes within about 1 cm, partly by luck.

Odometry agrees with Chrono to the millimetre, because the wheels do not slip on this floor. On a real
floor they slip a little, and the error of odometry grows with the distance driven.

### The fake board

@code
3. Summary
  wheel distances left / right    2.740 / 5.241 m
                                          x       y   heading
  end of the square, planned          0.000   0.000     0.0 deg
  end of the square, Chrono          -0.020   0.011    -1.8 deg
  end of the square, odometry        -0.020   0.010    -1.8 deg
  odometry error                  0.001 m, 0.0 deg
  messages sent / received        1406 / 1651
  round trip mean / max           15.2 / 37.4 ms
  largest wheel torque            1.135 N m
  stall torque of a motor at 12 V 0.653 N m
@endcode

With free motors the speeds follow the targets to within about 0.1 rad/s, and each turn ends within two
degrees of the plan. The round trip is mostly waiting: both sides talk every 20 ms, so a command waits for up
to 20 ms before the board reads it. On real WiFi, add a few milliseconds more.

The torque line is the most important result. To follow the free motors, the simulated wheels needed up
to 1.1 N m, at the starts and stops, while the motors stall at 0.65 N m. The free motors change speed far
faster than they could with the robot on them. Finding that out before building the robot is what
hardware in the loop is for. A slower ramp of the speed targets, or stronger motors, would fix it.

## Exercises

1. Stop the fake board while the example runs. What happens to the robot, and how long does it take for
   the simulation to notice? Where in the code is that decided?
2. Set `OBSERVER_BANDWIDTH` to 3 and to 30 rad/s. How do the largest torque and the odometry change?
3. Ramp the targets of `square_plan` up and down over half a second instead of stepping them. What is the
   largest torque now?
4. Double `speed_kp` in the robot file and rerun `--simulated-motors` and `--fake-board`. Which one gets
   better, and which one gets worse?
5. Lower the friction of the floor to 0.2. How large does the error of odometry get after one square?

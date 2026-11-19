\page bitrl_examples bitrl examples

# bitrl examples

The examples go through the library in order, from its basic types to robots that run on real
hardware, and each section uses what the sections before it showed. They live in
`examples/<section>/<example>/` and are built with the library when `ENABLE_EXAMPLES` is `ON`; the
executable has the name of the example. Some need optional dependencies, and the page of each
example says which. See \ref bitrl_install for how to build bitrl.

With `ENABLE_TESTS=ON`, `ctest -L examples` runs every example that needs no hardware, no server and
no window, and checks that it finishes without an error.

## 1. Basics

- \ref bitrl_example_basic_types

## 2. Frames and rotations

Where things are, and which way they face.

- \ref bitrl_example_frames
- \ref bitrl_example_rotation_matrices

## 3. Kinematics

How the joints of an arm, or the wheels of a robot, move what they carry.

- \ref bitrl_example_forward_kinematics
- \ref bitrl_example_inverse_kinematics
- \ref bitrl_example_diff_drive_kinematics

## 4. Trajectories

When to be where along a path.

- \ref bitrl_example_point_to_point

## 5. Motion planning

Finding a path that avoids the obstacles.

- \ref bitrl_example_a_star
- \ref bitrl_example_rrt

## 6. Sensors and actuators

What a robot measures with, and what it moves with.

- \ref bitrl_example_chrono_ultrasonic_sensor
- \ref bitrl_example_bldc_motor

## 7. Simulation

Simulating bodies and robots with Chrono. These need bitrl built with `ENABLE_CHRONO=ON`.

- \ref bitrl_example_chrono_rigid_body
- \ref bitrl_example_chrono_rigid_body_simulation
- \ref bitrl_example_chrono_smc_system
- \ref bitrl_example_chrono_diff_drive
- \ref bitrl_example_line_following_infrared_sonar

## 8. State estimation

What a robot can work out about itself from its sensors.

- \ref bitrl_example_imu_heading
- \ref bitrl_example_wheel_odometry
- \ref bitrl_example_kalman_filter
- \ref bitrl_example_extended_kalman_filter
- \ref bitrl_example_ekf_sensor_fusion
- \ref bitrl_example_ekf_sensor_fusion_mqtt

## 9. Hardware in the loop

Real boards and sensors, and the simulated robot with real parts: motors on a board, a camera on
a Raspberry Pi.

- \ref bitrl_example_arduino_usb
- \ref bitrl_example_arduino_wifi
- \ref bitrl_example_mqtt_ultrasonic_sensor
- \ref bitrl_example_mqtt_camera
- \ref bitrl_example_real_dc_motors
- \ref bitrl_example_pi_camera_line_following

## 10. Control

- \ref bitrl_example_quadrotor_dynamics
- \ref bitrl_example_quadrotor_rc_simulation

## Reinforcement learning

Environments for reinforcement learning. The Gymnasium environments run on the
<a href="https://github.com/pockerman/bitrl-rest-api">bitrl-rest-api</a> server, which the examples
talk to over HTTP.

- \ref bitrl_example_gymnasium_part_1
- \ref bitrl_example_gym_walk
- \ref bitrl_example_gymnasium_part_2
- \ref bitrl_example_gymnasium_part_3
- \ref bitrl_example_acrobot_vector_env
- \ref bitrl_example_grid_world
- \ref bitrl_example_connect2
- \ref bitrl_example_tensorboard
- \ref bitrl_example_chrono_rl_env

## The robotics course

The lessons of the robotics course are examples of the sections above. In the order of the course:

1. \ref bitrl_example_frames
2. \ref bitrl_example_rotation_matrices
3. \ref bitrl_example_forward_kinematics
4. \ref bitrl_example_inverse_kinematics
5. \ref bitrl_example_bldc_motor
6. \ref bitrl_example_real_dc_motors
7. \ref bitrl_example_pi_camera_line_following
8. \ref bitrl_example_line_following_infrared_sonar
9. \ref bitrl_example_imu_heading
10. \ref bitrl_example_wheel_odometry

\page bitsim_examples bitsim examples

# bitsim examples

The examples go through the library in order, from its basic types to robots that run on real
hardware, and each section uses what the sections before it showed. They live in
`examples/<section>/<example>/` and are built with the library when `ENABLE_EXAMPLES` is `ON`; the
executable has the name of the example. Some need optional dependencies, and the page of each
example says which. See \ref bitsim_install for how to build bitsim.

With `ENABLE_TESTS=ON`, `ctest -L examples` runs every example that needs no hardware, no server and
no window, and checks that it finishes without an error.

## 1. Basics {#bitsim_examples_basics}

- \ref bitsim_example_basic_types

## 2. Frames and rotations {#bitsim_examples_frames}

Where things are, and which way they face.

- \ref bitsim_example_frames
- \ref bitsim_example_rotation_matrices

## 3. Kinematics {#bitsim_examples_kinematics}

How the joints of an arm, or the wheels of a robot, move what they carry.

- \ref bitsim_example_forward_kinematics
- \ref bitsim_example_inverse_kinematics
- \ref bitsim_example_diff_drive_kinematics

## 4. Trajectories {#bitsim_examples_trajectories}

When to be where along a path.

- \ref bitsim_example_point_to_point

## 5. Motion planning {#bitsim_examples_planning}

Finding a path that avoids the obstacles.

- \ref bitsim_example_a_star
- \ref bitsim_example_rrt

## 6. Sensors and actuators {#bitsim_examples_sensors_actuators}

What a robot measures with, and what it moves with.

- \ref bitsim_example_chrono_ultrasonic_sensor
- \ref bitsim_example_bldc_motor

## 7. Simulation {#bitsim_examples_simulation}

Simulating bodies and robots with Chrono. These need bitsim built with `ENABLE_CHRONO=ON`.

- \ref bitsim_example_chrono_rigid_body
- \ref bitsim_example_chrono_rigid_body_simulation
- \ref bitsim_example_chrono_diff_drive
- \ref bitsim_example_line_following_infrared_sonar

## 8. State estimation {#bitsim_examples_estimation}

What a robot can work out about itself from its sensors.

- \ref bitsim_example_imu_heading
- \ref bitsim_example_wheel_odometry
- \ref bitsim_example_kalman_filter
- \ref bitsim_example_extended_kalman_filter
- \ref bitsim_example_ekf_sensor_fusion
- \ref bitsim_example_ekf_sensor_fusion_mqtt

## 9. Hardware in the loop {#bitsim_examples_hil}

Real boards and sensors, and the simulated robot with real parts: motors on a board, a camera on
a Raspberry Pi.

- \ref bitsim_example_arduino_usb
- \ref bitsim_example_arduino_wifi
- \ref bitsim_example_mqtt_ultrasonic_sensor
- \ref bitsim_example_mqtt_camera
- \ref bitsim_example_real_dc_motors
- \ref bitsim_example_pi_camera_line_following

## 10. Control {#bitsim_examples_control}

- \ref bitsim_example_quadrotor_dynamics
- \ref bitsim_example_quadrotor_rc_simulation
- \ref bitsim_example_fixed_wing_flight

## The robotics course {#bitsim_examples_course}

The lessons of the robotics course are examples of the sections above. In the order of the course:

1. \ref bitsim_example_frames
2. \ref bitsim_example_rotation_matrices
3. \ref bitsim_example_forward_kinematics
4. \ref bitsim_example_inverse_kinematics
5. \ref bitsim_example_bldc_motor
6. \ref bitsim_example_real_dc_motors
7. \ref bitsim_example_pi_camera_line_following
8. \ref bitsim_example_line_following_infrared_sonar
9. \ref bitsim_example_imu_heading
10. \ref bitsim_example_wheel_odometry

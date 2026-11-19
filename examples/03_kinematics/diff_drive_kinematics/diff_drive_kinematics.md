\page bitrl_example_diff_drive_kinematics The kinematics of a differential drive

A differential drive robot has two wheels on one axle, each with its own motor; it drives by
turning them at the same speed and turns by turning them at different speeds. Its state in the
plane is where the centre of its axle is and which way it faces, \f$(x, y, \theta)\f$, and it is
driven by its speed \f$v\f$ and its rate of turn \f$\omega\f$. This example drives it with
\ref bitrl::dynamics::DiffDriveDynamics "DiffDriveDynamics": straight, and then a quarter of a
circle.

## What you need

Nothing beyond bitrl itself. See \ref bitrl_install.

## The model

With a constant \f$v\f$ and \f$\omega\f$ the robot drives a circle of radius \f$v / \omega\f$.
Over a step of \f$\Delta t\f$ it drives an arc of length \f$d = v \Delta t\f$ that turns it by
\f$\phi = \omega \Delta t\f$:

\f[
x' = x + \frac{d}{\phi} (\sin\theta' - \sin\theta), \qquad
y' = y - \frac{d}{\phi} (\cos\theta' - \cos\theta), \qquad
\theta' = \theta + \phi
\f]

and with no turn, \f$\phi = 0\f$, it moves by \f$d\f$ along its heading. This is the default
version of the model, `DynamicVersion::V1`. Because each step is an arc, not a straight line, the
result does not depend on the length of the step.

The input is a map with `v` [m/s], `w` [rad/s] and `errors`: two numbers added to the distance
and to the turn of each step, to model what the wheels do wrong. `set_time_step` sets
\f$\Delta t\f$ and `integrate` moves the robot by one step. The second argument of the constructor
says whether to keep the Jacobians of the model up to date, which the extended Kalman filter of
\ref bitrl_example_extended_kalman_filter needs; here they are not needed.

\ref bitrl::estimation::DiffDriveOdometry "DiffDriveOdometry" in \ref bitrl_example_wheel_odometry
works the other way round: it works out the motion from how far the wheels turned.

## Running the example

@code{.sh}
./diff_drive_kinematics
@endcode

## Output

@code
1. Straight ahead
  start: x = 0.000 m, y = 0.000 m, theta = 0.0 deg
  after 2 s at 0.5 m/s: x = 1.000 m, y = 0.000 m, theta = 0.0 deg

2. A quarter of a circle to the left
  after 2 s at 45 deg/s: x = 1.637 m, y = 0.637 m, theta = 90.0 deg
  radius v / w [m]           0.637
  a quarter circle from (1, 0) facing x ends at (1.637, 0.637) facing y
@endcode

## Things to try

1. Make the step 0.5 s and 1 s. Does the end of the quarter circle move?
2. Turn to the right, with a negative \f$\omega\f$, for 8 s. Where does the robot end up, and what
   is its heading?
3. Add an error of 0.01 m to the distance of every step. How far off is the robot after a full
   circle?

## The code

\include diff_drive_kinematics.cpp

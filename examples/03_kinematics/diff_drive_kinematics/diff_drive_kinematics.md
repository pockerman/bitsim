\page bitrl_example_diff_drive_kinematics The kinematics of a differential drive

A differential drive robot has two wheels on one axle, each with its own motor; it drives by
turning them at the same speed and turns by turning them at different speeds. Its state in the
plane is where the centre of its axle is and which way it faces, \f$(x, y, \theta)\f$, and it is
driven by its speed \f$v\f$ and its rate of turn \f$\omega\f$. This example integrates that state
one step with \ref bitrl::dynamics::DiffDriveDynamics "DiffDriveDynamics".

## What you need

Nothing beyond bitrl itself. See \ref bitrl_install.

## The model

\ref bitrl::dynamics::DiffDriveDynamics "DiffDriveDynamics" takes the input as a map with `v`
[m/s], `w` [rad/s] and `errors`, two numbers added to the distance driven and to the heading to
model what the wheels do wrong. `set_time_step` sets the step \f$\Delta t\f$ and `integrate` moves
the state by one step. With `DynamicVersion::V1` and no turning the axle centre moves by
\f$\tfrac{1}{2} v \Delta t\f$ along the heading.

The second argument of the constructor says whether to keep the matrices of the linearised model
up to date, which the extended Kalman filter of \ref bitrl_example_extended_kalman_filter needs.
Here they are not needed.

\ref bitrl::estimation::DiffDriveOdometry "DiffDriveOdometry" in \ref bitrl_example_wheel_odometry
works the other way round: it works out the motion from how far the wheels turned.

## Running the example

@code{.sh}
./diff_drive_kinematics
@endcode

## Output

@code
Initial state:
X:0.0000
Y:0.0000
Theta:0.0000

X:0.0125
Y:0.0000
Theta:0.0000
@endcode

## The code

\include diff_drive_kinematics.cpp

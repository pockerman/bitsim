\page bitrl_example_point_to_point Point to point motions

A path says where to go; a trajectory says when to be where. This example moves the spatial arm of
\ref bitrl_example_forward_kinematics from one joint configuration to another in 2 s, with three
ways of timing the motion, and compares what they ask of the motors. Then it compares a straight
line in joint space with a straight line in the world.

## What you need

bitrl built with Chrono, for the arm's kinematics; see \ref bitrl_install.

## Time scaling

A point to point motion from \f$q_0\f$ to \f$q_1\f$ follows the straight line between them,

\f[
q(t) = q_0 + s(t)\,(q_1 - q_0), \qquad \dot q = \dot s\,(q_1 - q_0), \qquad
\ddot q = \ddot s\,(q_1 - q_0)
\f]

where the *time scaling* \f$s(t)\f$ goes from 0 at the start to 1 at the end, at time \f$T\f$. The
path is the same for every time scaling; only its timing changes. bitrl has three, in
`bitrl/trajectory/time_scaling.h`:

| Time scaling | \f$s(\tau)\f$, \f$\tau = t / T\f$ | starts and stops |
| ------------ | -------------------------------- | ---------------- |
| \ref bitrl::trajectory::CubicTimeScaling "CubicTimeScaling" | \f$3\tau^2 - 2\tau^3\f$ | at rest, but with a jump in acceleration |
| \ref bitrl::trajectory::QuinticTimeScaling "QuinticTimeScaling" | \f$10\tau^3 - 15\tau^4 + 6\tau^5\f$ | at rest and with no acceleration |
| \ref bitrl::trajectory::TrapezoidalTimeScaling "TrapezoidalTimeScaling" | constant acceleration, cruise, constant braking | at rest, with a jump in acceleration |

\ref bitrl::trajectory::PointToPoint "PointToPoint" moves between two points of any dimension, here
the three joint angles, with one of them.

The cubic is the simplest polynomial that starts and ends at rest. Its acceleration jumps from 0 to
\f$6/T^2\f$ when the motion starts: an infinite jerk, which shakes an arm and what it carries. The
quintic also starts and ends with no acceleration, so it is smooth, but it has to go faster in the
middle, \f$1.875/T\f$ against \f$1.5/T\f$. The trapezoid is what most motor controllers do: they
are given a top speed and an acceleration, and its top speed is the lowest of the three for a
given time.

## A straight line in the world

A straight line between two joint configurations is not a straight line for the tip of the arm:
the tip swings along a curve. To move the tip along a straight line in the world, the example takes
the point on the line at \f$s(t)\f$ and finds the joint angles for it with the inverse kinematics
of \ref bitrl_example_inverse_kinematics, at every step.

## Running the example

@code{.sh}
./point_to_point
@endcode

## Output

@code
1. Three ways to time the same motion
    t [s]   cubic s   ds/dt  d2s/dt2   quintic s   ds/dt  d2s/dt2   trapezoid s   ds/dt  d2s/dt2
     0.00     0.000   0.000    1.500       0.000   0.000    0.000       0.000   0.000    1.333
     0.25     0.043   0.328    1.125       0.016   0.179    1.230       0.042   0.333    1.333
     0.50     0.156   0.562    0.750       0.104   0.527    1.406       0.167   0.667    0.000
     0.75     0.316   0.703    0.375       0.275   0.824    0.879       0.333   0.667    0.000
     1.00     0.500   0.750    0.000       0.500   0.938    0.000       0.500   0.667    0.000
     1.25     0.684   0.703   -0.375       0.725   0.824   -0.879       0.667   0.667    0.000
     1.50     0.844   0.562   -0.750       0.896   0.527   -1.406       0.833   0.667    0.000
     1.75     0.957   0.328   -1.125       0.984   0.179   -1.230       0.958   0.333   -1.333
     2.00     1.000   0.000   -1.500       1.000   0.000    0.000       1.000   0.000   -1.333

2. On the arm
  motion       top joint speed   top joint accel   accel at t = 0   top jerk
                       [deg/s]         [deg/s^2]        [deg/s^2]  [deg/s^3]
  cubic                   97.5             195.0            195.0     195000
  quintic                121.9             187.6              0.0        974
  trapezoid               86.7             173.3            173.3     173333
  (the jerk is from steps of 1 ms: a jump in acceleration shows as a huge one)

3. A straight line in joint space is not one in the world
  tip at the start               (0.284, -0.493, 0.207)
  tip at the end                 (0.184, 0.507, 0.709)
  straight line [m]              1.123
  joint space: tip travels [m]   1.451
  joint space: off the line [m]  0.384
  world: off the line [m]        4.5e-07
  world: top joint speed [deg/s] 230.460
@endcode

The base joint turns by 130 degrees, the most of the three, and sets the top speeds. The cubic and
the trapezoid start with an acceleration of 195 and 173 deg/s² out of nothing; the quintic starts
with none, at the price of a top speed a quarter higher. With the joints on a straight line the
tip travels 1.45 m and passes 38 cm from the straight line between its ends. On the straight line
in the world it stays within a micrometre of it, but the joints have to turn faster, up to 230
deg/s: the line passes closer to the base, where the base has to turn quickly to follow it.

## Things to try

1. Make the trapezoid's acceleration share 0.5, a triangle. How do its top speed and acceleration
   compare with the cubic's?
2. Time the straight line in the world with the cubic instead of the quintic. What changes in the
   joint speeds?
3. Move the end point closer to the base. How fast does the base joint have to turn on the straight
   line in the world, and why?

## The code

\include point_to_point.cpp

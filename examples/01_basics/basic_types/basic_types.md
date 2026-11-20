\page bitsim_example_basic_types Basic types

Everything in bitsim is built from a few types and helpers: its number, vector and matrix types,
its constants, points, angles and units, and the JSON files that describe robots. This example
shows each of them; every other example uses them.

## What you need

Nothing beyond bitsim itself. See \ref bitsim_install.

## Numbers, vectors and matrices

`bitsim/bitsim_types.h` names the types bitsim works with. `real_t` is its floating point type, a
`double`, and `uint_t` its unsigned integer, a `std::size_t`. Vectors and matrices are
<a href="https://eigen.tuxfamily.org">Eigen</a>'s:

| Type | What it is |
| ---- | ---------- |
| `RealColVec3d` | a column vector of three `real_t`, `Eigen::Vector3d` |
| `RealMat3d` | a 3 x 3 matrix |
| `DynMat<T>`, `DynVec<T>`, `ColVec<T>` | a matrix, a row vector and a column vector whose size is set when they are made |
| `RealVec`, `RealColVec` | `DynVec<real_t>` and `ColVec<real_t>` |
| `STD_RealVec` | a `std::vector<real_t>` |

The examples about frames and kinematics use Chrono's `ChVector3d` and `ChMatrix33d` as well,
because Chrono simulates them; `ChMatrix33d` is an Eigen matrix too.

## Constants

`bitsim/bitsim_consts.h` has \f$\pi\f$, `consts::maths::PI`, and the acceleration of gravity,
`consts::maths::G`, a `TOLERANCE` for comparing numbers, `INVALID_ID` and `INVALID_STR` for what is
not set, and the directories with the robot, sensor and world files, `ROBOTS_DIR`, `SENSORS_DIR`
and `WORLDS_DIR`.

## Points

\ref bitsim::geometry::GeomPoint "GeomPoint<dim>" is a point in `dim` dimensions. It adds,
subtracts and scales like a vector, and knows its distance to other points, its norm and its dot
product.

## Angles and units

`bitsim/utils/unit_converter.h` converts degrees to radians and back, revolutions per minute to
rad/s, and the speed of a wheel to the speed of the robot. `maths::wrap_angle` wraps an angle
to \f$(-\pi, \pi]\f$: headings are compared after wrapping, since turning 350 degrees to the left
ends where turning 10 degrees to the right does.

## Robot files

The simulated robots of the later examples are described by JSON files in `ROBOTS_DIR`: the size
of the chassis and the wheels, the motors, and the sensors with their backends. The example reads
`bitsim_diff_drive_robot.json`, the robot of the line following examples, with the
<a href="https://github.com/nlohmann/json">nlohmann/json</a> library that bitsim ships.

## Running the example

@code{.sh}
./basic_types
@endcode

## Output

@code
1. Numbers, vectors and matrices
  real_t has 8 bytes, uint_t 8
  |v|                        3.000
  (A v)_x                    5.000
  M is 2 x 4
  sum of M                   4.000
  max of the samples         0.400

2. Constants
  PI                         3.142
  G [m/s^2]                  9.810
  TOLERANCE                  1.0e-08
  INVALID_ID is uint_t(-1)   true

3. Points
  a = ( 1,2 ), b = ( 4,6 ), a + b = ( 5,8 ), 2 a = ( 2,4 )
  distance from a to b       5.000
  |b - a|                    5.000
  a . b                      16.000

4. Angles and units
  90 deg [rad]               1.571
  1 rad [deg]                57.296
  60 rpm [rad/s]             6.283
  1 rad/s, 0.1 m wheel [m/s] 0.100
  350 deg wrapped [deg]      -10.000

5. Robot files
  bitsim_diff_drive_robot.json describes "Odysseus"
  wheel radius [m]           0.100
  axle length [m]            0.400
  motor gear ratio           30.000
  sensors: front-camera (downward-camera) front-sonar (ultrasonic-range) imu (imu) line-sensor (infrared-array) rear-sonar (ultrasonic-range)
@endcode

The labelled lines come from `bitsim/utils/io/print_utils.h`, which the other examples use too.

## The code

\include basic_types.cpp

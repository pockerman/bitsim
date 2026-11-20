\page bitsim_example_forward_kinematics Forward kinematics

This example accompanies the third lesson of the robotics course and builds on
\ref bitsim_example_frames and \ref bitsim_example_rotation_matrices. There we placed frames by
hand. Now the frames are attached to the links of a robot arm and they move when its joints move. You
need _bitsim_ configured with Chrono support to run it.

## What forward kinematics is

A robot arm is a chain of rigid links connected by joints. The joint angles
\f$q = (q_1, q_2, \dots, q_n)\f$ are what the motors control, but the task is usually described by the
position and orientation of the tool at the end of the arm, the **end-effector**. Forward kinematics
answers the question

> Given the joint angles, where is the end-effector?

The answer is a function \f$X_{WE} = f(q)\f$. It always exists and it is unique, which is not the case
for the opposite question. That is the topic of \ref bitsim_example_inverse_kinematics.

We keep the notation of the previous lessons. `X_AB` is the pose of frame \f$B\f$ expressed in frame
\f$A\f$, and the inner letters cancel. Frame \f$i\f$ is attached to joint \f$i\f$ and \f$E\f$ is the
end-effector.

## A joint is a frame that moves

Every joint in this lesson is revolute and turns around its own \f$z\f$ axis. Chrono motors use the same
convention. The frame of a joint has two parts: a fixed one that says where the joint is mounted, and a
rotation that depends on the joint angle.

\f[
X_{W1}(q) = X_{\text{mount}} \, R_z(q)
\f]

The arms and their kinematics are in bitsim, see \ref bitsim_kinematics; `joint_rotation` is
`bitsim::kinematics::joint_rotation`:

@code{.cpp}
ChFrame<> joint_rotation(real_t q) { return ChFrame<>(ChVector3d(0.0, 0.0, 0.0), rot_z(q)); }

const ChFrame<> X_W1 = mount * joint_rotation(q);
const ChVector3d tip_in_world = X_W1 * tip_in_link;
@endcode

The tip of a link of length 0.4 on a joint at \f$(1, 0, 0)\f$ moves on a circle of radius 0.4:

@code
1. A joint is a frame that moves
  tip at q = 0               (1.400, 0.000, 0.000)
  tip at q = 45              (1.283, 0.283, 0.000)
  tip at q = 90              (1.000, 0.400, 0.000)
  tip at q = 180             (0.600, 0.000, 0.000)
  distance tip-joint         0.400
@endcode

## A planar arm with two links

The first arm moves in the \f$xy\f$ plane, like a SCARA robot seen from above. It has two links of
lengths \f$l_1 = 0.4\f$ and \f$l_2 = 0.3\f$. Each link adds a fixed offset along its \f$x\f$ axis, and
each joint adds a rotation. Forward kinematics is just the product of the frames along the chain:

\f[
X_{WE} = R_z(q_1) \; T_x(l_1) \, R_z(q_2) \; T_x(l_2)
\f]

@code{.cpp}
const ChFrame<> X_W1 = joint_rotation(q1);
const ChFrame<> X_12 = ChFrame<>(ChVector3d(l1, 0.0, 0.0)) * joint_rotation(q2);
const ChFrame<> X_2E = ChFrame<>(ChVector3d(l2, 0.0, 0.0));
const ChFrame<> X_WE = X_W1 * X_12 * X_2E;
@endcode

For a planar arm this can also be written out with trigonometry. Each link points along the sum of the
joint angles before it:

\f[
x = l_1 \cos q_1 + l_2 \cos(q_1 + q_2) \qquad y = l_1 \sin q_1 + l_2 \sin(q_1 + q_2)
\f]

The two agree, and the orientation of the end-effector is the rotation \f$R_z(q_1 + q_2)\f$. At
\f$(30, 60)\f$ the tool points along the world \f$y\f$ axis.

@code
2. A planar arm with two links
      q1      q2   chain of frames             formula                     same
     0.0     0.0   (0.700, 0.000, 0.000)       (0.700, 0.000, 0.000)       true
    90.0     0.0   (0.000, 0.700, 0.000)       (0.000, 0.700, 0.000)       true
     0.0    90.0   (0.400, 0.300, 0.000)       (0.400, 0.300, 0.000)       true
    30.0    60.0   (0.346, 0.500, 0.000)       (0.346, 0.500, 0.000)       true
    45.0   -90.0   (0.495, 0.071, 0.000)       (0.495, 0.071, 0.000)       true
   120.0   150.0   (-0.200, 0.046, 0.000)      (-0.200, 0.046, 0.000)      true
  tool x axis at (30, 60)    (0.000, 1.000, 0.000)
@endcode

The formula is shorter, but it has to be derived again for every arm and it gets long very quickly. The
chain of frames works for any arm, and this is how Chrono and most robotics libraries compute it.

## The workspace

The **workspace** is the set of points the end-effector can reach. The example tries every pair of joint
angles on a one degree grid and records the distance of the tip from the first joint. For the planar arm
the workspace is a ring:

\f[
|l_1 - l_2| \le \sqrt{x^2 + y^2} \le l_1 + l_2
\f]

The arm reaches the outer circle when it is fully stretched and the inner one when it is fully folded.
Notice also that two different configurations, with the elbow on either side, put the tip at the same
point. Forward kinematics maps many joint angles to one pose, so going back is not unique.

@code
3. The workspace
  closest reach on the grid  0.100
  |l1 - l2|                  0.100
  furthest reach on the grid 0.700
  l1 + l2                    0.700
  tip with elbow on one side (0.400, 0.300, 0.000)
  tip with elbow on the other (0.400, 0.300, 0.000)
@endcode

## A spatial arm with three joints

The second arm leaves the plane. It is the structure of many industrial arms without their wrist:

- joint 1 turns the whole arm around the vertical axis, at height \f$h_0 = 0.3\f$
- joint 2 is the shoulder and joint 3 the elbow. Both are horizontal, so the upper arm \f$l_1\f$ and the
  forearm \f$l_2\f$ move in a vertical plane

The only new thing is the shoulder. Its axis must be horizontal, so its fixed part contains the rotation
\f$R_x(90^\circ)\f$. This turns the \f$z\f$ axis of joint 2 from vertical to horizontal, and the rest of
the chain is the planar arm again, now standing up.

@code{.cpp}
const ChFrame<> X_W1 = ChFrame<>(ChVector3d(0.0, 0.0, h0)) * joint_rotation(q[0]);
const ChFrame<> X_12 = ChFrame<>(ChVector3d(0.0, 0.0, 0.0), rot_x(90.0 * DEG)) * joint_rotation(q[1]);
const ChFrame<> X_23 = ChFrame<>(ChVector3d(l1, 0.0, 0.0)) * joint_rotation(q[2]);
const ChFrame<> X_3E = ChFrame<>(ChVector3d(l2, 0.0, 0.0));
const ChFrame<> X_WE = X_W1 * X_12 * X_23 * X_3E;
@endcode

Since the shoulder and the elbow move the arm in a vertical plane, the result can be checked with the
formula of the planar arm. It gives the reach \f$r\f$ and the height \f$z\f$ in that plane, and joint 1
turns the plane:

\f[
r = l_1 \cos q_2 + l_2 \cos(q_2 + q_3) \qquad z = h_0 + l_1 \sin q_2 + l_2 \sin(q_2 + q_3)
\f]
\f[
x = r \cos q_1 \qquad y = r \sin q_1
\f]

With \f$q = (90, 30, -60)\f$ the arm faces the world \f$y\f$ axis, the upper arm points up and the
forearm points down. The columns of \f$R_{WE}\f$ tell us that the tool looks forward and 30 degrees down,
which is \f$q_2 + q_3\f$.

@code
4. A spatial arm with three joints
  joint 1                    (0.000, 0.000, 0.300)
  joint 2 (shoulder)         (0.000, 0.000, 0.300)
  joint 3 (elbow)            (0.000, 0.346, 0.500)
  end-effector               (0.000, 0.606, 0.350)
  R_WE
      0.000   0.000   1.000 
      0.866   0.500   0.000 
     -0.500   0.866   0.000 
  from the formula           (0.000, 0.606, 0.350)
  same                       true
@endcode

## Denavit-Hartenberg parameters

Every arm so far was described with its own choice of fixed frames. The Denavit-Hartenberg (DH)
convention is a standard way to do this with only four numbers per joint. The frame of each joint is the
product

\f[
A_i = R_z(\theta_i) \; T_z(d_i) \; T_x(a_i) \; R_x(\alpha_i)
\f]

- \f$\theta_i\f$ the rotation around \f$z\f$. For a revolute joint this is the joint angle
- \f$d_i\f$ the offset along \f$z\f$
- \f$a_i\f$ the length of the link along the new \f$x\f$ axis
- \f$\alpha_i\f$ the twist of the next joint axis around the new \f$x\f$ axis

The arm is then a table, and forward kinematics multiplies the rows:

@code{.cpp}
ChFrame<> dh_frame(const DHRow &row)
{
    return joint_rotation(row.theta) * ChFrame<>(ChVector3d(row.a, 0.0, row.d), rot_x(row.alpha));
}
@endcode

The \f$\alpha = 90^\circ\f$ of joint 1 is the \f$R_x(90^\circ)\f$ we put in the shoulder by hand. For
this arm both descriptions give the same frame. In general they place the intermediate frames
differently, but the end-effector is always the same.

@code
5. Denavit-Hartenberg parameters
  joint   theta       d       a   alpha
      1  90.000   0.300   0.000  90.000
      2  30.000   0.000   0.400   0.000
      3 -60.000   0.000   0.300   0.000
  end-effector from DH       (0.000, 0.606, 0.350)
  same position              true
  same orientation           true
@endcode

@note Datasheets of industrial arms often give DH tables, so it is worth knowing how to read them. Be
careful though: there is also a "modified" DH convention that multiplies the four factors in a different
order. Check which one a table uses.

## Checking against a simulation

Finally we build the spatial arm in Chrono. The bodies are placed with the joint frames of our forward
kinematics at \f$q = 0\f$, and each joint is a `ChLinkMotorRotationAngle`. Like our joints, a Chrono
motor turns around the \f$z\f$ axis of the frame it is given.

@code{.cpp}
auto motor = chrono_types::make_shared<chrono::ChLinkMotorRotationAngle>();
motor->Initialize(child, parent, X_Wi);
motor->SetAngleFunction(chrono_types::make_shared<chrono::ChFunctionSine>(45.0 * DEG, 0.5));
@endcode

Each joint follows a sine wave and gravity acts on the links. Chrono knows nothing about our forward
kinematics: it solves the equations of motion of the bodies under the joint constraints. At every step we
read the joint angles from the motors, evaluate \f$f(q)\f$ and compare it with the tip of the simulated
forearm.

@code
6. Checking against a simulation
     t      q1      q2      q3   tip in simulation        tip from FK              error
  0.40    52.9    42.8   -50.7   (0.356, 0.471, 0.531)    (0.356, 0.471, 0.531)    1.1e-06
  0.80    85.6    26.5   -54.3   (0.048, 0.622, 0.338)    (0.048, 0.622, 0.338)    9.8e-07
  1.20    85.6   -26.5    -7.5   (0.047, 0.605, -0.046)   (0.047, 0.605, -0.046)   9.8e-07
  1.60    52.9   -42.8    46.2   (0.358, 0.473, 0.046)    (0.358, 0.473, 0.046)    1.5e-06
  2.00     0.0    -0.0    57.1   (0.563, 0.000, 0.552)    (0.563, 0.000, 0.552)    2.9e-06
  largest error              2.9e-06
@endcode

The remaining error is the tolerance of the simulation.

@warning The model uses `ChSolver::Type::SPARSE_QR`. With the default iterative solver of
`ChSystemNSC` the joints give a little under the weight of the links, and the error grows to tens of
centimetres. That is a property of the solver and not of the kinematics, but it is a useful reminder that
a simulation is a model too. Always check the accuracy of yours before trusting it.

## Running the example

@code
./forward_kinematics
./forward_kinematics --render
@endcode

With `--render` the arm of section 6 keeps moving in an Irrlicht window. The frames of the joints and of
the end-effector are drawn from our forward kinematics, with \f$x\f$ in red, \f$y\f$ in green and
\f$z\f$ in blue, while the bodies are where Chrono puts them. Each joint turns around the blue axis of
its own frame. Joints 1 and 2 share their origin, so the label 2 is drawn a little above it. The yellow
line is the path of the tip over the last few seconds.

The panel at the top right shows the joint angles, the tip from forward kinematics and from Chrono, and
the distance between the two. Watch how the frame of joint 3 is carried along by joint 2, and the frame
of \f$E\f$ by both: this is the chain \f$X_{W1} X_{12} X_{23} X_{3E}\f$ at work.

## Exercises

1. Add a fourth joint to the spatial arm: a wrist at the end of the forearm, parallel to the elbow, with
   a short link of 0.1. How does the orientation of the end-effector depend on \f$q_2 + q_3 + q_4\f$?
2. Write the DH table of the planar arm and check it against `planar_fk_frames`.
3. Change the order of the factors in `dh_frame` to the modified convention,
   \f$R_x(\alpha_{i-1}) \, T_x(a_{i-1}) \, R_z(\theta_i) \, T_z(d_i)\f$, and write the table of the
   spatial arm for it.
4. Plot the workspace of the planar arm when joint 2 can only move between \f$-90^\circ\f$ and
   \f$90^\circ\f$. Is it still a ring?

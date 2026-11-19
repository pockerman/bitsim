\page bitrl_example_bldc_motor A brushless DC motor

This example accompanies the fifth lesson of the robotics course. In \ref bitrl_example_forward_kinematics
and \ref bitrl_example_inverse_kinematics the joints of the arm were ideal: they went exactly where we told
them. Here we look at what actually turns a joint, an electric motor, and we put one into the shoulder of
the arm. You need _bitrl_ configured with Chrono support to run it.

## How a brushless motor works

Most robot joints, drones and electric vehicles use **brushless DC motors** (BLDC). The rotor carries
permanent magnets, and the stator carries three coils, the **phases** \f$a\f$, \f$b\f$ and \f$c\f$. A
current in the coils makes a magnetic field that pulls the magnets around. To keep the rotor turning, the
field has to move ahead of the rotor, so the current has to be switched from one pair of phases to the
next as the rotor turns. This is called **commutation**.

A brushed DC motor does this mechanically, with brushes sliding over a segmented ring on the shaft. A
brushless motor has no brushes: the controller measures the rotor angle and switches the phases
electronically. There is nothing to wear out, and the coils, which get hot, sit on the outside where
they can be cooled.

Two equations describe the motor. A current \f$i\f$ produces a torque, and a turning rotor produces a
voltage, the **back-EMF**, that opposes the supply:

\f[
T = K_t \, i \qquad e = K_e \, \omega
\f]

In SI units \f$K_t\f$ in N m/A and \f$K_e\f$ in V s/rad are the same number. This is energy conservation:
the electrical power \f$e \, i\f$ that goes into the back-EMF comes out as the mechanical power
\f$T \omega\f$.

## What the motor constants tell us

The example uses a small motor of the size found in robot joints:

| parameter | symbol | value |
|---|---|---|
| resistance between two terminals | \f$R\f$ | 0.6 \f$\Omega\f$ |
| inductance between two terminals | \f$L\f$ | 0.4 mH |
| torque and back-EMF constant | \f$K_t = K_e\f$ | 0.05 N m/A |
| rotor inertia | \f$J\f$ | \f$2 \cdot 10^{-5}\f$ kg m\f$^2\f$ |
| viscous friction | \f$b\f$ | \f$10^{-5}\f$ N m s/rad |
| pole pairs | \f$p\f$ | 7 |
| supply voltage | \f$V\f$ | 24 V |

Averaged over the commutation, the motor behaves like a brushed DC motor, and we start with that model.
It has an electrical and a mechanical equation:

\f[
L \frac{di}{dt} = V - R \, i - K_e \, \omega \qquad
J \frac{d\omega}{dt} = K_t \, i - b \, \omega - T_{\text{load}}
\f]

Without simulating anything, these give the numbers on a datasheet. At standstill there is no back-EMF
and the current is only limited by the resistance: the **stall current** \f$V / R\f$ and the
**stall torque** \f$K_t V / R\f$. Without a load the motor speeds up until the back-EMF almost cancels the
supply, at the **no-load speed** \f$\approx V / K_e\f$. In between, the torque falls linearly with the
speed, and the mechanical power peaks at half the no-load speed.

@code
1. What the motor constants tell us
  no-load speed                     478.851 rad/s
  no-load speed                        4573 rpm
  stall current                        40.0 A
  stall torque                        2.000 N m
  electrical time constant L/R        0.667 ms
  mechanical time constant            4.800 ms

    speed    torque   current     power
  (rad/s)     (N m)       (A)       (W)
      0.0     2.000      40.0       0.0
    120.0     1.500      30.0     180.0
    240.0     1.000      20.0     240.0
    360.0     0.500      10.0     180.0
    480.0     0.000       0.0       0.0
@endcode

The two time constants tell us how fast the motor responds. The current settles in about
\f$\tau_e = L / R\f$, and the speed in about \f$\tau_m = J R / (K_t K_e)\f$. Here the mechanical one is
seven times slower. Notice that \f$\tau_m\f$ depends on the electrical constants: the back-EMF acts as a
brake that grows with the speed.

## Switching the motor on

The models of this example are in bitrl: \ref bitrl::actuators::BLDCMotorParameters "BLDCMotorParameters",
`dc_equivalent_step`, \ref bitrl::actuators::BLDCMotor "BLDCMotor",
\ref bitrl::actuators::BLDCRotor "BLDCRotor" and the speed controller,
\ref bitrl::control::PIController "PIController".

We integrate the DC model with the explicit Euler method. The step of 1 \f$\mu\f$s is more than 600
times smaller than the fastest time constant, which is what makes such a simple method accurate here.

@code{.cpp}
DCEquivalentState dc_equivalent_step(const BLDCMotorParameters &m, const DCEquivalentState &s,
                                     real_t V, real_t T_load, real_t dt)
{
    const real_t di = (V - m.R * s.i - m.Ke * s.w) / m.L;
    const real_t dw = (m.Kt() * s.i - m.b * s.w - T_load) / m.J;
    return {s.i + dt * di, s.w + dt * dw};
}
@endcode

@code
2. Switching the motor on
   t (ms)   current (A)   speed (rad/s)   torque (N m)
      0.2        10.353             2.7          0.518
      0.5        20.846            14.7          1.042
      1.0        29.550            47.1          1.477
      2.0        31.474           125.9          1.574
      5.0        17.093           308.0          0.855
     10.0         4.982           430.0          0.249
     20.0         0.494           474.9          0.025
     50.0         0.096           478.8          0.005

  peak current                         32.1 A
  time to 63% of final speed          4.875 ms
  final speed, simulated            478.849 rad/s
  final speed, predicted            478.851 rad/s
@endcode

The current jumps to over 30 A, almost the stall current, and falls as the motor speeds up and the
back-EMF grows. The inductance only delays the jump by a fraction of a millisecond. The speed reaches
63% of its final value after about \f$\tau_m\f$, and ends where the constants predict. A real
motor controller limits the current, since 32 A would quickly overheat a motor like this one.

## Electronic commutation

Now we open up the commutation. The back-EMF of each phase depends on the rotor angle. Here it is a sine
wave, shifted by 120 degrees from phase to phase. What matters is the **electrical angle**
\f$\theta_e = p \, \theta\f$: with 7 pole pairs the pattern repeats 7 times per mechanical turn.

The simplest scheme is **six-step commutation**. At any time the current flows in through one phase and
out through another, and the third is disconnected. The controller picks the pair whose back-EMF
difference is the largest at the current rotor angle. That divides an electrical turn into six sectors
of 60 degrees.

To know the sector, three **Hall sensors** on the stator detect the magnets of the rotor. Each one is on
for half an electrical turn, and they are 120 degrees apart, so together they give a 3-bit code that
changes every 60 degrees. The controller looks the code up in a table:

@code{.cpp}
switch (hall_code)
{
case 4: return {2, 1}; // c to b, rotor at -30 to 30 electrical degrees
case 5: return {0, 1}; // a to b, 30 to 90
case 1: return {0, 2}; // a to c, 90 to 150
case 3: return {1, 2}; // b to c, 150 to 210
case 2: return {1, 0}; // b to a, 210 to 270
case 6: return {2, 0}; // c to a, 270 to 330
default: return {0, 0};
}
@endcode

@code
3. Electronic commutation
  angle (el. deg)   Hall code   current flows
                0           4   c to b
               30           5   a to b
               60           5   a to b
               90           1   a to c
              120           1   a to c
              150           3   b to c
              180           3   b to c
              210           2   b to a
              240           2   b to a
              270           6   c to a
              300           6   c to a
              330           4   c to b
@endcode

The codes 0 and 7 never happen with working sensors. A real controller treats them as a fault.

## The three-phase model

In each sector the two active phases are in series, so the circuit has the resistance and the inductance
between two terminals, and the difference of the two back-EMFs:

\f[
L \frac{di}{dt} = V - R \, i - k(\theta_e) \, \omega \qquad T = k(\theta_e) \, i
\f]

where \f$k(\theta_e)\f$ is the back-EMF difference of the active pair per unit of speed. This is the DC
model with a constant that changes with the rotor angle. Within a sector it goes through its peak
\f$k_{\text{line}}\f$, and at the edges it has dropped to \f$k_{\text{line}} \cos 30^\circ\f$. Its average
over a sector is \f$\frac{3}{\pi} k_{\text{line}}\f$, and this average is the \f$K_e\f$ of the DC model.

The class \ref bitrl::actuators::BLDCMotor "BLDCMotor" implements this. It does not integrate the motion of the rotor: it takes the angle
and the speed as inputs and returns the torque. That way the same class works with our own model of the
rotor here, and with Chrono in section 6.

@code{.cpp}
real_t step(real_t V, real_t theta, real_t w, real_t dt)
{
    const real_t theta_e = m_.pole_pairs * theta;
    pair_ = commutation(hall_code(sensed_angle(theta)));

    const real_t k = phase_emf(pair_.high, theta_e) - phase_emf(pair_.low, theta_e);
    i_ += dt * (V - m_.R * i_ - k * w) / m_.L;
    return k * i_;
}
@endcode

@note We assume that the current moves to the new pair of phases instantly when the sector changes. In
a real motor the inductance makes this take a few microseconds, during which the current in the old
phase decays through the diodes of the drive. That is a second source of torque ripple, which we leave
out.

With a load of 0.5 N m the three-phase model runs at the same mean speed and draws the same mean current
as the DC model, which is why the DC model is so widely used. The difference is the **torque ripple**:
the torque dips at the edges of every sector, by about \f$1 - \cos 30^\circ \approx 13\%\f$. That is felt
as vibration and heard as noise, at a frequency of six times the electrical speed.

@code
4. The three-phase model
  with a load of 0.500 N m      three-phase     DC model
  mean speed (rad/s)                 359.1        359.1
  mean current (A)                   10.07        10.07
  torque min / mean / max     0.458 / 0.504 / 0.528
  torque ripple (max-min)/mean        0.140 

  sensor error (el. deg)   torque per ampere (N m/A)   cos(error)
                       0                      0.0500       1.0000
                      15                      0.0483       0.9659
                      30                      0.0433       0.8660
                      60                      0.0249       0.5000
                      90                      0.0000       0.0000
                     180                     -0.0500      -1.0000
@endcode

The second table shows why the Hall sensors must be in the right place. With an error, the controller
switches too early or too late, and every sector is shifted away from the peak of the back-EMF. The
torque per ampere drops as the cosine of the error: at 90 degrees the motor makes no torque at all, and at
180 it runs backwards. The torque is measured with the rotor turned slowly at a constant speed, so that
the current hardly changes. Notice that the error is in electrical degrees. With 7 pole pairs, 15
electrical degrees are only about 2 mechanical degrees.

## Speed control

To run at a given speed the controller adjusts the voltage. In a real drive the transistors switch the
supply on and off thousands of times per second, and the motor sees the average. This is called
**pulse width modulation** (PWM). We simulate the average voltage directly, limited to \f$\pm 24\f$ V.
A negative voltage swaps the high and low side of the pair, and drives the motor the other way.

A PI controller computes the voltage from the speed error. It is tuned on the DC model, where the
response of the speed to the voltage is \f$\frac{1/K_e}{\tau_m s + 1}\f$. Choosing \f$k_i = k_p / \tau_m\f$
cancels the mechanical time constant, and \f$k_p = K_e \tau_m / \tau_c\f$ gives a closed loop with time
constant \f$\tau_c = 1\f$ ms. We also feed forward the voltage that balances the back-EMF at the desired
speed, \f$K_e \omega_{\text{ref}}\f$. The PI controller then only has to correct the remaining error, and
does not have to build up 15 V in its integral after every change of the setpoint.

@code{.cpp}
const real_t feedforward = use_feedforward ? m.Ke * setpoint(t) : 0.0;
V = controller.update(setpoint(t) - rotor.w, steps_per_control * dt, feedforward);
@endcode

The controller runs at 10 kHz and the motor model at 1 MHz. The motor first runs at 300 rad/s, then a
load of 0.3 N m is applied from 40 to 80 ms, and then the motor reverses to -300 rad/s:

@code
5. Speed control
   t (ms)   setpoint   speed (rad/s)   voltage (V)   current (A)   load (N m)
       10        300           298.8          15.3          0.25         0.00
       20        300           299.9          15.0          0.05         0.00
       30        300           300.0          15.0          0.05         0.00
       40        300           300.0          15.0          0.05         0.00
       50        300           298.1          18.6          6.27         0.30
       60        300           299.8          18.6          6.09         0.30
       70        300           300.0          18.6          6.07         0.30
       80        300           300.1          18.6          6.06         0.30
       90       -300          -291.5         -15.1          0.37         0.00
      100       -300          -299.1         -15.0         -0.16         0.00
      110       -300          -299.9         -15.0         -0.08         0.00
      120       -300          -300.0         -15.0         -0.07         0.00
      130       -300          -300.0         -15.0         -0.06         0.00
      140       -300          -300.0         -15.0         -0.06         0.00
      150       -300          -300.0         -15.0         -0.05         0.00
      160       -300          -300.0         -15.0         -0.05         0.00

  controller gain kp                 0.2400 V s/rad
  speed error from 20 to 40 ms         0.09 rad/s
@endcode

Under load the controller raises the voltage by \f$R \, i\f$ to push the extra current through the coils,
and the speed returns to the setpoint. The current printed is the average over the last millisecond, since
the instantaneous current ripples from sector to sector. At the start and at the reversal the voltage is
at its limit for a few milliseconds, and the response is slower than \f$\tau_c\f$. The controller can not
ask for more than the supply can give, and it stops integrating while the output is limited so that it
does not overshoot afterwards. This is called **anti-windup**.

## A motor in the joint of an arm

Finally we put the motor into the shoulder of the arm of \ref bitrl_example_forward_kinematics, with the
upper arm, 0.4 m long and 1 kg, and no forearm. A motor turns much faster than a joint and makes much
less torque than it needs, so it drives the joint through a gearbox with ratio \f$N = 20\f$. The motor
turns \f$N\f$ times faster than the joint and the joint gets \f$N\f$ times the torque:

\f[
\theta_{\text{motor}} = N q \qquad T_{\text{joint}} = N \, T_{\text{motor}}
\f]

The gearbox also multiplies the inertia of the rotor by \f$N^2\f$, as seen from the joint. That is 15% of
the inertia of the link here, so the example adds it to the link.

The joint is now a `ChLinkMotorRotationTorque`. Chrono simulates the arm, and our motor model provides
the torque. This is a small **co-simulation**: the two models run side by side and exchange values at
every step. Chrono steps every millisecond, and the motor runs 100 steps in between with the speed of the
rotor held constant:

@code{.cpp}
real_t motor_torque = 0.0;
for (int k = 0; k < motor_substeps; ++k)
{
    const real_t theta = joint.gear_ratio * (q + k * dt * q_dot);
    const real_t w = joint.gear_ratio * q_dot;
    motor_torque += (motor.step(V, theta, w, dt) - m.b * w) / motor_substeps;
}
torque->SetConstant(joint.gear_ratio * motor_torque);
system.DoStepDynamics(step_size);
@endcode

The arm starts horizontal and has to go to 45 degrees. A PI controller computes the voltage from the
angle error. We need no derivative term because the back-EMF already damps the motion: seen from the
joint it acts as a damper of \f$N^2 K_t K_e / R \approx 1.7\f$ N m s/rad.

@code
6. A motor in the joint of an arm
  PI controller
   t (s)   angle (deg)   voltage (V)   current (A)   joint torque (N m)
     0.2          58.1         -2.19         -1.22               -1.114
     0.4          44.8          1.09          1.77                1.623
     0.6          45.4          0.75          1.33                1.397
     0.8          45.1          0.81          1.33                1.365
     1.0          45.0          0.80          1.33                1.386
     1.2          45.0          0.79          1.33                1.387
     1.4          45.0          0.79          1.33                1.387
     1.6          45.0          0.79          1.32                1.387
     1.8          45.0          0.79          1.32                1.387
     2.0          45.0          0.79          1.32                1.387

  P controller, after 2 s
  angle                               42.54 deg
  current                              1.43 A

  Holding the arm at 45.00 deg
  torque of gravity at the joint      1.387 N m
  torque at the motor                0.0694 N m
  current needed, at least            1.325 A
  current needed, at most             1.530 A
  current in the simulation           1.325 A
@endcode

The arm overshoots by more than 10 degrees and then settles at 45. To hold it there, the motor has to
balance the weight of the link, \f$m g \frac{l}{2} \cos q\f$, with a constant current. A motor at
standstill makes no back-EMF, so the voltage is just \f$R \, i\f$, and only 0.8 V are needed.

Without the integral term the arm stops 2.5 degrees short. The proportional term only produces a voltage
when there is an error, and here it needs about 0.9 V to hold the arm up. The integral term builds up
that voltage and removes the error.

At standstill the torque per ampere depends on where in its sector the rotor stopped, between
\f$k_{\text{line}} \cos 30^\circ\f$ and \f$k_{\text{line}}\f$. So the holding current can be anywhere
between the two values printed. The simulated current is inside that range, near the lower end, because
the rotor stopped near the middle of a sector.

The ideal joints of the previous lessons were a good start. With a motor the arm takes time to move,
overshoots and needs a controller. Those are the effects a controller design has to deal with.

## Running the example

@code
./bldc_motor
./bldc_motor --render
@endcode

With `--render` the arm of section 6 is given a new target every 2.5 s: 45, 90, 0, -45 and 135 degrees.
The white line points at the target, and the panel shows the voltage, the current and the torque at the
joint. The simulation runs four times slower than real time, since the motor is too fast to follow
otherwise.

On the left is a schematic of the motor over one electrical turn. The yellow arrow is the electrical angle
of the rotor, \f$p N q\f$. The phase the current goes into is red, the phase it comes out of is blue, and
the disconnected phase is grey, exactly as the commutation table of section 3 says. When the arm moves
the colours switch every 60 degrees of the arrow. When it holds still they stop, and the current has to
keep flowing to hold the arm up. Try to see how much harder the motor works at 0 degrees, where the arm is
horizontal, than at 90 degrees, where it hangs on the joint.

## Exercises

1. Set `use_feedforward` to `false` in section 5. What happens after each change of the setpoint, and
   why?
2. Add a current limit of 10 A to the switch-on of section 2: when the current reaches 10 A, reduce the
   voltage to keep it there. How much longer does the motor take to reach full speed?
3. Lift the arm of section 6 to 90 degrees, and then to 135 degrees. How do the voltage and the current
   needed to hold it change?
4. Use a gear ratio of 5 and then of 100 in section 6. Explain the change in the response using the
   reflected inertia \f$N^2 J\f$ and the damping \f$N^2 K_t K_e / R\f$.
5. Replace six-step commutation with sinusoidal commutation: apply a voltage to all three phases,
   proportional to their back-EMF shapes. You will need a model with three currents. What happens to the
   torque ripple?

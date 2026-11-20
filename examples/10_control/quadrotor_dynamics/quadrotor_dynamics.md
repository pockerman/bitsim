\page bitsim_example_quadrotor_dynamics The dynamics of a quadrotor

A quadrotor is controlled by the speeds of its four rotors: each pushes up with a thrust
\f$k\,\omega_i^2\f$ and turns the body the other way with a drag torque \f$\beta\,\omega_i^2\f$.
Equal speeds lift or lower it; differences tilt and turn it, and a tilted thrust moves it sideways.
This example simulates \ref bitsim::dynamics::QuadrotorDynamics "QuadrotorDynamics" for 2 s with the
four rotors at the same speed, so that it only climbs and sinks. It is the model that a controller
for a quadrotor, simulated or real, starts from.

## What you need

Nothing beyond bitsim itself; Python with `matplotlib` for the plots. See \ref bitsim_install.

## The model

The state has twelve values: the position \f$(x, y, z)\f$ in the world, with \f$z\f$ down as in
aircraft, the velocity \f$(u, v, w)\f$ and the angular velocity \f$(p, q, r)\f$ in the body's
axes, and the Euler angles \f$(\phi, \theta, \psi)\f$. The rotors are in a plus: 1 in front, 2
on the right, 3 behind and 4 on the left. A faster left rotor rolls the quadrotor right, a
faster front rotor pitches its nose up, and rotors 1 and 3 spin the other way to 2 and 4, so
speeding them up yaws it. The class documentation has the equations. The parameters are those of the quadrotor in
<a href="https://sal.aalto.fi/publications/pdf-files/eluu11_public.pdf">Luukkonen, *Modelling and
control of quadcopter*</a>: a mass of 0.468 kg, arms of 0.225 m, \f$k\f$ = 2.98e-6 and
\f$\beta\f$ = 1.14e-7.

Four rotors at 625 rad/s push with \f$4k\omega^2\f$ = 4.66 N, a little more than the weight of
4.59 N. The example raises the speeds to 675 rad/s in the first 0.1 s, lowers them to 550 rad/s by
0.4 s, and brings them back to 625 rad/s at 0.5 s, all four together.

## Running the example

@code{.sh}
./quadrotor_dynamics
python plot.py      # plots state.csv
@endcode

It takes steps of 0.1 ms, prints the height every 0.1 s, and writes the whole state, with the
rotor speeds, to `state.csv`; its `height` column is \f$-z\f$.

## Output

@code
t = 0 s, height 1.39252e-09 m, climbing at 1.39252e-05 m/s, rotors at 625 rad/s
t = 0.1 s, height 0.00341272 m, climbing at 0.0957384 m/s, rotors at 674.994 rad/s
t = 0.2 s, height 0.0196106 m, climbing at 0.20497 m/s, rotors at 633.328 rad/s
t = 0.3 s, height 0.0399304 m, climbing at 0.179784 m/s, rotors at 591.662 rad/s
t = 0.4 s, height 0.0513729 m, climbing at 0.0290241 m/s, rotors at 549.996 rad/s
t = 0.5 s, height 0.0473708 m, climbing at -0.0715539 m/s, rotors at 625 rad/s
...
t = 1 s, height 0.0290038 m, climbing at -0.00192779 m/s, rotors at 625 rad/s
...
t = 2 s, height 0.0967091 m, climbing at 0.137324 m/s, rotors at 625 rad/s
@endcode

The faster rotors lift it, and it keeps climbing as they slow down until they are below the hover
speed, 0.23 s into the run. By 0.4 s it has climbed 5 cm and sinks again. From 0.5 s the
rotors are back at 625 rad/s, a little faster than the 620.6 rad/s that carries the weight, and
the quadrotor stops sinking at 1 s and climbs ever faster: by 0.139 m/s each second, which is
\f$4 k \omega^2 / m - g\f$. It never tilts: with four equal rotors there is no torque.

## Things to try

1. Find the speed at which it hovers, \f$\omega = \sqrt{m g / 4k}\f$, and check it.
2. Make one pair of opposite rotors faster than the other pair. Which way does it turn, and why?
3. Write a PID controller for the height, with the rotor speed as its output, using
   \ref bitsim::control::PIDController "PIDController".

## The code

\include quadrotor_dynamics.cpp

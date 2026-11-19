\page bitrl_example_quadrotor_dynamics The dynamics of a quadrotor

A quadrotor is controlled by the speeds of its four rotors: each pushes up with a thrust
\f$k\,\omega_i^2\f$ and turns the body the other way with a drag torque \f$\beta\,\omega_i^2\f$.
Equal speeds lift or lower it; differences tilt and turn it, and a tilted thrust moves it sideways.
This example simulates \ref bitrl::dynamics::QuadrotorDynamics "QuadrotorDynamics" for 2 s with the
four rotors at the same speed, which should only lift or lower it. It is the model that a
controller for a quadrotor, simulated or real, starts from.

## What you need

Nothing beyond bitrl itself; Python with `matplotlib` for the plots. See \ref bitrl_install.

## The model

The state has twelve values: the position \f$(x, y, z)\f$, with \f$z\f$ down as in aircraft, the
velocity \f$(u, v, w)\f$ in the body's axes, the angular velocity \f$(p, q, r)\f$ and the Euler
angles \f$(\phi, \theta, \psi)\f$. The parameters are those of the quadrotor in
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

It prints the time and the position at every step of 0.1 ms and writes the whole state, with the
rotor speeds, to `state.csv`.

## Things to try

1. Find the speed at which it hovers, \f$\omega = \sqrt{m g / 4k}\f$, and check it.
2. Make one pair of opposite rotors faster than the other pair. Which way does it turn, and why?
3. Write a PID controller for the height, with the rotor speed as its output, using
   \ref bitrl::control::PIDController "PIDController".

## The code

\include quadrotor_dynamics.cpp

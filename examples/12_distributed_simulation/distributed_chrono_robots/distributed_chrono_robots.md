\page bitsim_example_distributed_chrono_robots Chrono robots that see each other's ghosts

\ref bitsim_example_distributed_robots split a simulation of kinematic robots into participants.
Here every participant runs a full Chrono simulation of bitsim's differential drive robot, in a
Chrono system of its own. Two robots start 4 m apart, facing each other, and drive forward until
their front sonar hears something closer than 0.4 m.

The robot of the other participant is not in this participant's Chrono system: another process,
maybe on another machine, simulates it. What this participant has is the state of the other's
chassis, every 0.02 s. \ref bitsim::distributed::CHRONO_Ghosts "CHRONO_Ghosts" turns that into a
*ghost*: a fixed box the size of the chassis, moved to where the chassis is. Chrono does not move
it, but it collides, so the sonar's rays hit it like any other body.

## What you need

bitsim built with Chrono; with MQTT too, to run the robots as separate processes. See
\ref bitsim_install.

## A participant

@code{.cpp}
Synchroniser sync(transport, "robot_0", config); // config.peers = {"robot_1"}
CHRONO_Ghosts ghosts(system);
for (uint_t s = 0; ...; ++s)
{
    // send our chassis, and place the ghost of the other robot
    ghosts.update(sync.exchange(s, {chrono_body_state("chassis", *robot.get_chassis(), size)}));
    // two Chrono steps of 0.01 s, reading the sonar before each
}
@endcode

## Running the example

@code{.sh}
./distributed_chrono_robots              # both robots, as two threads
./distributed_chrono_robots --no-ghosts  # the same without ghosts
./distributed_chrono_robots --transport mqtt --participant 0 &
./distributed_chrono_robots --transport mqtt --participant 1
@endcode

## Output

@code
Each robot sees the ghost of the other
  robot_0: stopped after 7.700 s, its sonar reads 0.40 m, the front of its chassis is at x = -0.21 m
  robot_1: stopped after 7.700 s, its sonar reads 0.40 m, the front of its chassis is at x = +0.21 m
  the gap between the two fronts is 0.42 m

Without ghosts
  robot_0: never stopped, its sonar reads 4.00 m, the front of its chassis is at x = +1.96 m
  robot_1: never stopped, its sonar reads 4.00 m, the front of its chassis is at x = -1.96 m
  the gap between the two fronts is -3.91 m: in one world they would have crashed into each other
@endcode

With ghosts, each robot hears the other and stops 0.4 m from it: the gap between the fronts is
0.42 m, the two stopping distances less the little they rolled on. Without ghosts, each robot is
alone in its world, its sonar hears nothing, and the two drive straight through each other.
Run as two MQTT processes, in lockstep, the robots do exactly what the two threads do, to the last
bit of every number in `chrono_robot_0.csv` and `chrono_robot_1.csv`.

## What ghosts can and cannot do

A ghost shows where the other robot is, and nothing else: it does not push back. When two robots of
different participants collide, each participant sees its own robot hit a wall that does not
give. That is fine for robots that sense and avoid each other, as here. Robots that push each other,
or push the same box, need the bodies they push in one participant.

The robots exchange their state every 0.02 s, so a ghost is up to 0.02 s behind its robot: at
0.3 m/s, 6 mm. In wall-clock mode the ghosts are moved forward by their age, see
\ref bitsim_example_hardware_in_the_loop_sync.

The example sets Chrono's solver as \ref bitsim_example_line_following_infrared_sonar does, which
drives the same robot. With Chrono's default solver, and a step of 0.002 s, the chassis tipped
forward by about 3 degrees as it drove, and the sonar, which looks straight ahead along the chassis,
heard the floor 3 to 4 m away.

## Things to try

1. Give the robots different speeds. Where do they stop?
2. Exchange every 0.2 s instead of every 0.02 s. How much closer do they get?
3. Add a third robot that crosses between them.

## The code

\include distributed_chrono_robots.cpp

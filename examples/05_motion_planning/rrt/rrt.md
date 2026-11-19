\page bitrl_example_rrt Planning a path with a random tree

\ref bitrl_example_a_star cut the floor into cells and searched them. That gets slow when there are
many dimensions, such as the six joints of an arm: the number of cells grows with the power of the
dimension. A *rapidly exploring random tree* (RRT) does not cut the space up: it samples it. This
example plans across the warehouse of \ref bitrl_example_a_star with one.

## What you need

Nothing beyond bitrl itself. See \ref bitrl_install.

## The tree

\ref bitrl::planning::RRT "RRT" grows a tree from the start. At each iteration it

1. picks a random point of the warehouse, or the goal itself with a small probability, the *goal
   bias*, 5 % here;
2. finds the node of the tree nearest to that point;
3. grows a branch from that node towards the point, at most a *step* long, 20 cm here, if the
   branch is free;

and it stops when a new node is close enough to the goal and the last stretch to the goal is free.
The path is then the branches from the root to that node.

The tree needs only one thing about the world: whether a point is free. Here a point is free if the
robot's centre there is at least its radius from every shelf and wall, worked out from the
rectangles directly. A branch is free if its points are, checked every 2 cm.

Random branches zigzag, so the path is long. \ref bitrl::planning::RRT::shortcut "shortcut" takes
most of the zigzag out: from each point of the path it goes straight to the furthest later point
that a free segment reaches.

## Running the example

@code{.sh}
./rrt
@endcode

## Output

@code
1. A tree, its path, and the path shortened
  384 iterations, 219 nodes; the path has 62 points, shortened 6

  |                 ##                     |
  |     .           ##                     |
  |     .           ##     .  ###       G  |
  |  . .            ##   .... ###      **  |
  |  ...        .. .##   . .  ###     **.  |
  |   . . ###     . ## . .. . ###     *.   |
  |    ...###     .   .... .  ###    **..  |
  |     . ###     . .......   ###   **.    |
  |   . . ###     *****....   ###   *.     |
  |  . ...### ..  *.   *****. ###  **..    |
  |    .. ### ....*.##    .***    ** .     |
  |    .. ###    **.##   .  .**.  * ...    |
  |   . ..###    *..##  ..   ..****..      |
  |    .. ###    * .##  ..    . **. .      |
  | .. .. ###   **..##  .            .     |
  |  ..   ###   *.. ##   .  ########.      |
  |   ... ###   *.. ##      ########       |
  | ...   ###   * . ##      ########       |
  |  ..   ###  *... ##                     |
  | ..    ###  *....##                     |
  |  .        .*.   ##                     |
  |  ....*******    ##                     |
  | .S****.. .. .   ##                     |
  |  ..  ..         ##                     |
  |                 ##                     |

  path of the tree  12.14 m
  shortened         10.39 m
  A* on the grid    9.70 m

2. Every seed finds another path
  seed  iterations   nodes   path [m]   shortened [m]
     1        1657     517      13.44           10.67
     2         633     224      12.40           10.21
     3         321     192      12.01           10.15
     4         345     207      11.68           10.32
     5         507     218      11.20           10.00

3. The length of the branches
  step [m]  iterations   nodes   path [m]
      0.05        2514    1152      12.09
      0.20         384     219      12.14
      0.50         363     163      12.87
      1.00          98      45      11.21
@endcode

Each character is 20 cm: `#` a shelf or the wall, `.` a node of the tree, `*` the shortened path.
The tree spreads over the whole warehouse before a branch finds the door. Its path is 12.1 m, and
shortened 10.4 m, against 9.7 m for A* on a grid of the same free space. A* is shorter here than in
\ref bitrl_example_a_star because the free space is worked out from the rectangles, so the grown
corners are round, not square.

Every seed grows a different tree and finds a different path: an RRT is quick, and finds *a* path,
not the shortest. Short branches explore carefully but need many iterations; long ones grow fast,
but near obstacles most of them are blocked.

## Things to try

1. Set the goal bias to 0 and to 0.5. How many iterations does the tree need, and why can a high
   bias be slower?
2. Close the door. What does the tree do until it gives up?
3. RRT* rewires the tree to make paths shorter as it grows. Where in `plan()` would it go?

## The code

\include rrt.cpp

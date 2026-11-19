\page bitrl_example_a_star Planning a path with A*

A robot has to cross a warehouse, past shelves and through the door of a wall, without touching
anything. This example cuts the floor into cells, marks the ones the robot cannot be in, and
searches the free ones for the shortest path with A*.

## What you need

Nothing beyond bitrl itself. See \ref bitrl_install.

## The occupancy grid

\ref bitrl::planning::OccupancyGrid "OccupancyGrid" cuts the floor, 8 x 5 m, into cells of 10 cm,
each free or occupied. A robot is not a point: its centre has to stay its radius, 25 cm, away from
every shelf and wall. So the obstacles are grown by that radius before they go on the grid, and
then the robot can be planned as a point: any free cell is a place where its centre can be.

From a cell the robot can move to its eight neighbours, the four that share a side and the four
that share a corner, but not diagonally past the corner of an occupied cell.

## A*

`bitrl::planning::a_star` searches a graph from a start node to a goal. It keeps a list
of open nodes, ordered by

\f[
f(n) = g(n) + w\,h(n)
\f]

the cost \f$g\f$ of the best path to \f$n\f$ found so far plus the weighted *heuristic* \f$h\f$, a
guess of the cost from \f$n\f$ to the goal, and always expands the open node with the smallest
\f$f\f$. Here the cost of a move is the distance between the cell centres and the heuristic is the
straight line to the goal, which never guesses too high. With such a heuristic and \f$w = 1\f$ the
first path to reach the goal is a shortest one.

- \f$w = 0\f$ is Dijkstra's algorithm: it expands nodes in order of their distance from the start,
  in every direction, and finds a shortest path too, after more work;
- \f$w > 1\f$ trusts the heuristic more: it heads for the goal and expands fewer nodes, but the
  path may be longer.

`a_star` works on any graph: it takes the start, the goal, and functions for the neighbours, the
cost and the heuristic. `bitrl::planning::a_star_grid` is A* on an
OccupancyGrid.

## Running the example

@code{.sh}
./a_star
@endcode

## Output

@code
1. The warehouse and the shortest path
  80 x 50 cells of 0.1 m, 1927 of them too close to an obstacle

  ::::::::::::::::::#:::::::::::::::::::::
  ::              ::#:      ::::::       :
  ::              ::#:      ::##::     G :
  ::    ::::::    ::#:      ::##::     * :
  ::    ::::::    ::#:      ::##::    *  :
  ::    ::##::    ::#:      ::##::    *  :
  ::    ::##::    ::::      ::##::   *   :
  ::    ::##::              ::##::   *   :
  ::    ::##::   ********   ::##::  **   :
  ::    ::##::   *::::   *  ::##:: **    :
  ::    ::##::   *::#:    * ::::::**     :
  ::    ::##::   *::#:     ********      :
  ::    ::##::   *::#:                   :
  ::    ::##::   *::#:    ::::::::::     :
  ::    ::##::   *::#:    ::::::::::     :
  ::    ::##::   *::#:    :########:     :
  ::    ::##::   *::#:    :########:     :
  ::    ::##::   *::#:    ::::::::::     :
  ::    ::##::  **::#:    ::::::::::     :
  ::    ::##:: ** ::#:                   :
  ::    ::::::**  ::#:                   :
  ::      *****   ::#:                   :
  ::S******       ::#:                   :
  ::::::::::::::::::#:::::::::::::::::::::
  ::::::::::::::::::#:::::::::::::::::::::

2. How the searches compare
  search                        length   expanded
  A*                           10.76 m      1297 cells expanded
  Dijkstra (weight 0)          10.76 m      2031 cells expanded
  weighted A* (weight 3)       12.67 m       807 cells expanded
  A*, moves along the axes     12.40 m      1520 cells expanded

3. A robot that is too wide for the door
  A*, robot radius 0.45 m   no path
@endcode

Each character is 2 x 2 cells: `#` a shelf or the wall, `:` where the robot's centre cannot be,
`*` the path. The path hugs the grown shelf and goes through the door, 80 cm wide. Dijkstra finds
a path as short and expands half as many cells again; weighted A* expands fewer, but its path is 2
m longer. With moves along the axes only, the robot has to zigzag, and the path is 15 % longer. A
robot with a radius of 45 cm does not fit through the door, and A* says there is no path.

## Things to try

1. Move the door down to the floor. Does the path change, and does Dijkstra still expand more cells?
2. Try weights between 1 and 3. When does the path start to get longer?
3. Use a heuristic that guesses too high, `2.0 *` the distance, in `a_star` directly. Is the path
   still the shortest?
4. Make the cells 5 cm. How do the number of cells expanded and the time grow?

## The code

\include a_star.cpp

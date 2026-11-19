#ifndef BITRL_PLANNING_MODULE_H
#define BITRL_PLANNING_MODULE_H

/**
 * @defgroup bitrl_planning bitrl::planning
 * @brief Motion planning: finding a path from a start to a goal that avoids obstacles.
 *
 * - OccupancyGrid: a map of the plane cut into square cells that are free or occupied;
 * - a_star(): A* search on any graph, and a_star_grid() on an OccupancyGrid. With a heuristic
 *   weight of zero it is Dijkstra's algorithm;
 * - RRT: a rapidly exploring random tree, which samples the free space instead of cutting it
 *   into cells.
 */

#endif // BITRL_PLANNING_MODULE_H

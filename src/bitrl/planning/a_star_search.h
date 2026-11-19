#ifndef BITRL_PLANNING_A_STAR_SEARCH_H
#define BITRL_PLANNING_A_STAR_SEARCH_H

#include "bitrl/bitrl_types.h"
#include "bitrl/planning/occupancy_grid.h"

#include <algorithm>
#include <functional>
#include <limits>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

namespace bitrl
{
namespace planning
{

///
/// @ingroup bitrl_planning
/// @brief What a search found
///
template <typename Node> struct SearchResult
{
    bool found{false};      ///< whether the goal was reached
    std::vector<Node> path; ///< from the start to the goal, both included; empty if not found
    real_t cost{0.0};       ///< the cost of the path
    uint_t n_expanded{0};   ///< how many nodes the search took off its open list
};

/**
 * @ingroup bitrl_planning
 * @brief A* search from start to goal.
 *
 * The search keeps a list of open nodes ordered by \f$f(n) = g(n) + w\,h(n)\f$: the cost
 * \f$g\f$ of the best path to n found so far, plus the weighted heuristic \f$h\f$, a guess of
 * the cost from n to the goal. It expands the node with the smallest f, until that is the goal.
 * With an admissible heuristic, one that never guesses too high, and w = 1 the path found is a
 * shortest one. With w = 0 the search is Dijkstra's algorithm, which expands more nodes; with
 * w > 1 it expands fewer, but the path may be longer.
 *
 * @param start Where to start
 * @param goal Where to go
 * @param neighbours neighbours(n) gives the nodes next to n, as a container of Node
 * @param cost cost(a, b) is the cost of the move from a to b, which are neighbours
 * @param heuristic heuristic(n, goal) guesses the cost from n to the goal
 * @param weight The weight w of the heuristic
 *
 * Node needs ==, and std::hash<Node>.
 */
template <typename Node, typename Neighbours, typename Cost, typename Heuristic>
SearchResult<Node> a_star(const Node &start, const Node &goal, Neighbours neighbours, Cost cost,
                          Heuristic heuristic, real_t weight = 1.0)
{
    struct Open
    {
        real_t f;
        real_t g;
        Node node;
        bool operator>(const Open &other) const { return f > other.f; }
    };

    std::priority_queue<Open, std::vector<Open>, std::greater<Open>> open;
    std::unordered_map<Node, real_t> best_g;
    std::unordered_map<Node, Node> came_from;

    SearchResult<Node> result;
    best_g[start] = 0.0;
    open.push({weight * heuristic(start, goal), 0.0, start});

    while (!open.empty())
    {
        const Open current = open.top();
        open.pop();

        // A node can be on the list more than once, with the costs of the paths found to it.
        // Only the cheapest counts
        if (current.g > best_g[current.node])
        {
            continue;
        }
        ++result.n_expanded;

        if (current.node == goal)
        {
            result.found = true;
            result.cost = current.g;
            for (Node n = goal;; n = came_from.at(n))
            {
                result.path.push_back(n);
                if (n == start)
                {
                    break;
                }
            }
            std::reverse(result.path.begin(), result.path.end());
            return result;
        }

        for (const Node &next : neighbours(current.node))
        {
            const real_t g = current.g + cost(current.node, next);
            const auto known = best_g.find(next);
            if (known == best_g.end() || g < known->second)
            {
                best_g[next] = g;
                came_from.insert_or_assign(next, current.node);
                open.push({g + weight * heuristic(next, goal), g, next});
            }
        }
    }
    return result;
}

///
/// @ingroup bitrl_planning
/// @brief A* on an occupancy grid, moving to the neighbours of a cell (see
/// OccupancyGrid::neighbours) with the distance between cell centres as the cost and the
/// straight-line distance to the goal as the heuristic, which never guesses too high
///
SearchResult<GridCell> a_star_grid(const OccupancyGrid &grid, const GridCell &start,
                                   const GridCell &goal, bool diagonal = true, real_t weight = 1.0);

} // namespace planning
} // namespace bitrl

#endif // BITRL_PLANNING_A_STAR_SEARCH_H

#ifndef BITSIM_PLANNING_RRT_H
#define BITSIM_PLANNING_RRT_H

#include "bitsim/bitsim_types.h"

#include <array>
#include <functional>
#include <random>
#include <vector>

namespace bitsim
{
namespace planning
{

/**
 * @class RRT
 * @ingroup bitsim_planning
 * @brief A rapidly exploring random tree in the plane.
 *
 * The tree starts at the start. At each iteration it picks a random point of the bounds, or
 * the goal with probability goal_bias, finds the node of the tree nearest to it, and grows a
 * branch of at most step towards it, if the branch is free. It stops when a new node is within
 * goal_radius of the goal and the last stretch to the goal is free, or after max_iterations.
 *
 * The tree samples the free space instead of cutting it into cells, so it needs only a test of
 * whether a point is free, and it works as well in many dimensions as in two. Its paths are not
 * the shortest, and each run with a different seed finds a different one.
 */
class RRT
{
  public:
    using Point = std::array<real_t, 2>;

    ///
    /// @brief Whether a point is free of obstacles
    ///
    using FreeTest = std::function<bool(const Point &)>;

    struct Parameters
    {
        real_t step{0.2};            ///< the longest branch [m]
        real_t goal_bias{0.05};      ///< the probability of growing towards the goal
        real_t goal_radius{0.2};     ///< close enough to the goal [m]
        uint_t max_iterations{5000}; ///< before giving up
        real_t collision_step{0.02}; ///< how finely a branch is checked [m]
    };

    ///
    /// @brief A node of the tree: where it is, and the index of its parent; the root is its own
    /// parent
    ///
    struct Node
    {
        Point p;
        uint_t parent;
    };

    ///
    /// @param lower The lower left corner of the space to sample [m]
    /// @param upper The upper right corner [m]
    /// @param is_free The test of a point
    /// @param parameters How to grow the tree
    /// @param seed For the random points, so that a run can be repeated
    ///
    RRT(const Point &lower, const Point &upper, FreeTest is_free, const Parameters &parameters,
        uint_t seed = 42);

    ///
    /// @brief Grow a tree from start to goal. Returns whether it reached the goal
    ///
    bool plan(const Point &start, const Point &goal);

    ///
    /// @brief The tree of the last plan()
    ///
    const std::vector<Node> &tree() const noexcept { return tree_; }

    ///
    /// @brief The path of the last plan() from the start to the goal, both included; empty if
    /// the goal was not reached
    ///
    std::vector<Point> path() const;

    ///
    /// @brief The iterations the last plan() took
    ///
    uint_t n_iterations() const noexcept { return n_iterations_; }

    ///
    /// @brief Whether the straight segment from a to b is free, checked every collision_step
    ///
    bool is_segment_free(const Point &a, const Point &b) const;

    ///
    /// @brief A shorter path through the same free space: from each point of the path, go
    /// straight to the furthest later point that a free segment reaches. RRT paths zigzag, and
    /// this takes most of the zigzag out
    ///
    std::vector<Point> shortcut(const std::vector<Point> &path) const;

    ///
    /// @brief The length of a path [m]
    ///
    static real_t length(const std::vector<Point> &path);

  private:
    uint_t nearest_(const Point &p) const;

    Point lower_;
    Point upper_;
    FreeTest is_free_;
    Parameters p_;
    std::mt19937 rng_;
    std::vector<Node> tree_;
    uint_t goal_node_{0};
    bool reached_{false};
    uint_t n_iterations_{0};
};

} // namespace planning
} // namespace bitsim

#endif // BITSIM_PLANNING_RRT_H

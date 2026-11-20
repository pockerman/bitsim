#include "bitsim/bitsim_types.h"
#include "bitsim/planning/a_star_search.h"
#include "bitsim/planning/occupancy_grid.h"
#include "bitsim/planning/rrt.h"

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <vector>

namespace
{
using bitsim::real_t;
using namespace bitsim::planning;

// A 20 x 20 grid of 0.1 m cells with a wall across the middle and a gap at the top
OccupancyGrid wall_with_a_gap()
{
    OccupancyGrid grid(20, 20, 0.1);
    grid.add_rectangle(0.95, 0.0, 1.05, 1.6);
    return grid;
}
} // namespace

TEST(TestOccupancyGrid, CellsAndWorld)
{
    OccupancyGrid grid(10, 5, 0.2, -1.0, -0.5);
    EXPECT_EQ(grid.to_cell(-1.0, -0.5), (GridCell{0, 0}));
    EXPECT_EQ(grid.to_cell(0.99, 0.49), (GridCell{9, 4}));
    EXPECT_FALSE(grid.in_bounds(grid.to_cell(1.01, 0.0)));
    const auto p = grid.to_world({2, 1});
    EXPECT_NEAR(p[0], -0.5, 1.0e-12);
    EXPECT_NEAR(p[1], -0.2, 1.0e-12);
    EXPECT_THROW(OccupancyGrid(0, 1, 1.0), std::invalid_argument);
}

TEST(TestOccupancyGrid, ObstaclesAndNeighbours)
{
    OccupancyGrid grid(5, 5, 1.0);
    EXPECT_EQ(grid.neighbours({2, 2}).size(), 8u);
    EXPECT_EQ(grid.neighbours({2, 2}, false).size(), 4u);
    EXPECT_EQ(grid.neighbours({0, 0}).size(), 3u);

    // An occupied cell to the right also blocks the two diagonals past its corners
    grid.set_occupied({3, 2});
    EXPECT_FALSE(grid.is_free({3, 2}));
    EXPECT_EQ(grid.neighbours({2, 2}).size(), 5u);

    grid.add_circle(0.5, 4.5, 0.6);
    EXPECT_FALSE(grid.is_free({0, 4}));
    EXPECT_TRUE(grid.is_free({1, 3}));
    EXPECT_EQ(grid.n_occupied(), 2u);
}

TEST(TestAStar, FindsTheShortestPathAroundAWall)
{
    const auto grid = wall_with_a_gap();
    const GridCell start = grid.to_cell(0.25, 0.25);
    const GridCell goal = grid.to_cell(1.75, 0.25);

    const auto result = a_star_grid(grid, start, goal);
    ASSERT_TRUE(result.found);
    EXPECT_EQ(result.path.front(), start);
    EXPECT_EQ(result.path.back(), goal);
    for (std::size_t i = 1; i < result.path.size(); ++i)
    {
        EXPECT_TRUE(grid.is_free(result.path[i]));
        EXPECT_LE(std::abs(result.path[i].col - result.path[i - 1].col), 1);
        EXPECT_LE(std::abs(result.path[i].row - result.path[i - 1].row), 1);
    }

    // Dijkstra finds a path as short, and expands more cells to do it
    const auto dijkstra = a_star_grid(grid, start, goal, true, 0.0);
    ASSERT_TRUE(dijkstra.found);
    EXPECT_NEAR(dijkstra.cost, result.cost, 1.0e-9);
    EXPECT_GT(dijkstra.n_expanded, result.n_expanded);
}

TEST(TestAStar, NoPathThroughAClosedWall)
{
    OccupancyGrid grid(10, 10, 1.0);
    grid.add_rectangle(4.5, 0.0, 5.5, 10.0);
    EXPECT_FALSE(a_star_grid(grid, {1, 1}, {8, 8}).found);
    EXPECT_FALSE(a_star_grid(grid, {5, 5}, {8, 8}).found); // starts in the wall
}

TEST(TestAStar, WorksOnAnyGraph)
{
    // 0 - 1 - 3 costs 2 + 2, 0 - 2 - 3 costs 1 + 5
    const std::vector<std::vector<int>> edges{{1, 2}, {3}, {3}, {}};
    const auto cost = [](int a, int b)
    {
        if (a == 0)
        {
            return b == 1 ? 2.0 : 1.0;
        }
        return a == 1 ? 2.0 : 5.0;
    };
    const auto result = a_star(
        0, 3, [&edges](int n) { return edges[n]; }, cost, [](int, int) { return 0.0; });
    ASSERT_TRUE(result.found);
    EXPECT_EQ(result.path, (std::vector<int>{0, 1, 3}));
    EXPECT_DOUBLE_EQ(result.cost, 4.0);
}

TEST(TestRRT, ReachesTheGoalThroughTheGap)
{
    const auto grid = wall_with_a_gap();
    const auto is_free = [&grid](const RRT::Point &p)
    { return grid.is_free(grid.to_cell(p[0], p[1])); };
    RRT rrt({0.0, 0.0}, {2.0, 2.0}, is_free, {});

    ASSERT_TRUE(rrt.plan({0.25, 0.25}, {1.75, 0.25}));
    const auto path = rrt.path();
    ASSERT_GE(path.size(), 3u);
    EXPECT_DOUBLE_EQ(path.front()[0], 0.25);
    EXPECT_DOUBLE_EQ(path.back()[0], 1.75);
    for (std::size_t i = 1; i < path.size(); ++i)
    {
        EXPECT_TRUE(rrt.is_segment_free(path[i - 1], path[i]));
    }
    // It has to go up through the gap, so it is longer than the straight line
    EXPECT_GT(RRT::length(path), 2.0);

    // A shortcut path is no longer, and still free
    const auto shorter = rrt.shortcut(path);
    EXPECT_LE(RRT::length(shorter), RRT::length(path) + 1.0e-12);
    EXPECT_LE(shorter.size(), path.size());
    for (std::size_t i = 1; i < shorter.size(); ++i)
    {
        EXPECT_TRUE(rrt.is_segment_free(shorter[i - 1], shorter[i]));
    }

    // The same seed grows the same tree
    RRT again({0.0, 0.0}, {2.0, 2.0}, is_free, {});
    again.plan({0.25, 0.25}, {1.75, 0.25});
    EXPECT_EQ(again.tree().size(), rrt.tree().size());
}

TEST(TestRRT, GivesUpWhenThereIsNoWay)
{
    OccupancyGrid grid(20, 20, 0.1);
    grid.add_rectangle(0.95, 0.0, 1.05, 2.0);
    const auto is_free = [&grid](const RRT::Point &p)
    { return grid.is_free(grid.to_cell(p[0], p[1])); };
    RRT::Parameters parameters;
    parameters.max_iterations = 500;
    RRT rrt({0.0, 0.0}, {2.0, 2.0}, is_free, parameters);
    EXPECT_FALSE(rrt.plan({0.25, 0.25}, {1.75, 0.25}));
    EXPECT_TRUE(rrt.path().empty());
    EXPECT_EQ(rrt.n_iterations(), 500u);
}

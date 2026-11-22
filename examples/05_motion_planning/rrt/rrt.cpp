// Planning a path across the warehouse of the a_star example with a rapidly exploring random
// tree. The tree does not cut the floor into cells: it grows branches into random points of the
// free space until one reaches the goal. The example draws the tree and its path, shortens the
// path, and compares runs with different seeds and branch lengths with the path of A*.
//
// Usage: rrt

#include "bitsim/planning/rrt.h"
#include "bitsim/bitsim_types.h"
#include "bitsim/planning/a_star_search.h"
#include "bitsim/planning/occupancy_grid.h"
#include "bitsim/utils/concurrency/parallel.h"
#include "bitsim/utils/io/print_utils.h"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

namespace rrt
{

using bitsim::real_t;
using bitsim::uint_t;
using namespace bitsim::planning;

// The warehouse of the a_star example
const real_t WIDTH = 8.0;
const real_t HEIGHT = 5.0;
const real_t CELL = 0.1;
const real_t ROBOT_RADIUS = 0.25;

struct Box
{
    real_t x0, y0, x1, y1;
};
const std::vector<Box> OBSTACLES{
    {1.5, 1.0, 2.0, 4.0}, {3.5, 0.0, 3.7, 3.0}, {3.5, 3.8, 3.7, 5.0},
    {5.0, 1.5, 6.5, 2.0}, {5.5, 3.0, 6.0, 4.5},
};
const RRT::Point START{0.5, 0.5};
const RRT::Point GOAL{7.5, 4.5};

// Free if the robot's centre is at least ROBOT_RADIUS from every obstacle and wall. The tree
// needs only this test, so the obstacles stay rectangles: there is no grid
bool is_free(const RRT::Point &p)
{
    if (p[0] < ROBOT_RADIUS || p[0] > WIDTH - ROBOT_RADIUS || p[1] < ROBOT_RADIUS ||
        p[1] > HEIGHT - ROBOT_RADIUS)
    {
        return false;
    }
    for (const auto &b : OBSTACLES)
    {
        const real_t dx = std::max({b.x0 - p[0], 0.0, p[0] - b.x1});
        const real_t dy = std::max({b.y0 - p[1], 0.0, p[1] - b.y1});
        if (std::hypot(dx, dy) < ROBOT_RADIUS)
        {
            return false;
        }
    }
    return true;
}

// The warehouse, 0.2 m a character: # an obstacle, . a node of the tree, * the path
void draw(const RRT &tree, const std::vector<RRT::Point> &path)
{
    const int cols = static_cast<int>(WIDTH / 0.2);
    const int rows = static_cast<int>(HEIGHT / 0.2);
    std::vector<std::string> canvas(rows, std::string(cols, ' '));
    const auto put = [&](const RRT::Point &p, char ch)
    {
        const int c = std::min(cols - 1, static_cast<int>(p[0] / 0.2));
        const int r = std::min(rows - 1, static_cast<int>(p[1] / 0.2));
        canvas[r][c] = ch;
    };
    for (int r = 0; r < rows; ++r)
    {
        for (int c = 0; c < cols; ++c)
        {
            const RRT::Point centre{(c + 0.5) * 0.2, (r + 0.5) * 0.2};
            for (const auto &b : OBSTACLES)
            {
                if (centre[0] >= b.x0 && centre[0] <= b.x1 && centre[1] >= b.y0 &&
                    centre[1] <= b.y1)
                {
                    canvas[r][c] = '#';
                }
            }
        }
    }
    for (const auto &node : tree.tree())
    {
        put(node.p, '.');
    }
    // The path, every 5 cm along each segment
    for (std::size_t i = 1; i < path.size(); ++i)
    {
        const real_t d = std::hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1]);
        const int n = std::max(1, static_cast<int>(d / 0.05));
        for (int k = 0; k <= n; ++k)
        {
            const real_t t = static_cast<real_t>(k) / n;
            put({path[i - 1][0] + t * (path[i][0] - path[i - 1][0]),
                 path[i - 1][1] + t * (path[i][1] - path[i - 1][1])},
                '*');
        }
    }
    put(START, 'S');
    put(GOAL, 'G');
    for (int r = rows - 1; r >= 0; --r)
    {
        std::cout << "  |" << canvas[r] << "|\n";
    }
}

void run()
{
    // A* on the same warehouse, to compare with
    OccupancyGrid grid(static_cast<uint_t>(WIDTH / CELL), static_cast<uint_t>(HEIGHT / CELL), CELL);
    for (uint_t row = 0; row < grid.n_rows(); ++row)
    {
        for (uint_t col = 0; col < grid.n_cols(); ++col)
        {
            const GridCell c{static_cast<bitsim::int_t>(col), static_cast<bitsim::int_t>(row)};
            const auto p = grid.to_world(c);
            if (!is_free({p[0], p[1]}))
            {
                grid.set_occupied(c);
            }
        }
    }
    const auto a_star =
        a_star_grid(grid, grid.to_cell(START[0], START[1]), grid.to_cell(GOAL[0], GOAL[1]));

    bitsim::utils::io::print_title("1. A tree, its path, and the path shortened");
    RRT tree({0.0, 0.0}, {WIDTH, HEIGHT}, is_free, {});
    if (!tree.plan(START, GOAL))
    {
        std::cout << "  the tree did not reach the goal\n";
        return;
    }
    const auto path = tree.path();
    const auto shorter = tree.shortcut(path);
    std::cout << "  " << tree.n_iterations() << " iterations, " << tree.tree().size()
              << " nodes; the path has " << path.size() << " points, shortened " << shorter.size()
              << "\n\n";
    draw(tree, shorter);
    std::cout << std::fixed << std::setprecision(2) << "\n  path of the tree  " << RRT::length(path)
              << " m\n  shortened         " << RRT::length(shorter) << " m\n  A* on the grid    "
              << a_star.cost << " m\n";

    bitsim::utils::io::print_title("2. Every seed finds another path");
    std::cout << "  seed  iterations   nodes   path [m]   shortened [m]\n";
    // The runs share nothing but the map, which they only read, so they plan on several threads
    // at once. parallel_map gives the results back in the order of the seeds.
    struct SeedRun
    {
        uint_t seed;
        bool found;
        uint_t n_iterations, n_nodes;
        real_t length, shortened;
    };
    const auto seed_runs = bitsim::utils::concurrency::parallel_map(
        5,
        [&](std::size_t i)
        {
            const uint_t seed = i + 1;
            RRT run({0.0, 0.0}, {WIDTH, HEIGHT}, is_free, {}, seed);
            if (!run.plan(START, GOAL))
            {
                return SeedRun{seed, false, run.n_iterations(), run.tree().size(), 0.0, 0.0};
            }
            return SeedRun{seed,
                           true,
                           run.n_iterations(),
                           run.tree().size(),
                           RRT::length(run.path()),
                           RRT::length(run.shortcut(run.path()))};
        });
    for (const auto &r : seed_runs)
    {
        if (!r.found)
        {
            std::cout << std::setw(6) << r.seed << "  no path\n";
            continue;
        }
        std::cout << std::setw(6) << r.seed << std::setw(12) << r.n_iterations << std::setw(8)
                  << r.n_nodes << std::setw(11) << r.length << std::setw(16) << r.shortened << "\n";
    }

    bitsim::utils::io::print_title("3. The length of the branches");
    std::cout << "  step [m]  iterations   nodes   path [m]\n";
    for (const real_t step : {0.05, 0.2, 0.5, 1.0})
    {
        RRT::Parameters parameters;
        parameters.step = step;
        RRT run({0.0, 0.0}, {WIDTH, HEIGHT}, is_free, parameters);
        const bool found = run.plan(START, GOAL);
        std::cout << std::setw(10) << step << std::setw(12) << run.n_iterations() << std::setw(8)
                  << run.tree().size();
        if (found)
        {
            std::cout << std::setw(11) << RRT::length(run.path());
        }
        else
        {
            std::cout << "    no path";
        }
        std::cout << "\n";
    }
}

} // namespace rrt

int main()
{
    rrt::run();
    return 0;
}

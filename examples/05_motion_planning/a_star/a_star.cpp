// Planning a path for a robot across a warehouse with A*. The floor is cut into cells of 10 cm,
// the shelves and walls are grown by the robot's radius so that the robot can be planned as a
// point, and A* searches the free cells. The example compares A* with Dijkstra's algorithm, with
// a weighted heuristic, and with moves along the axes only.
//
// Usage: a_star

#include "bitsim/bitsim_types.h"
#include "bitsim/planning/a_star_search.h"
#include "bitsim/planning/occupancy_grid.h"
#include "bitsim/utils/io/print_utils.h"

#include <iomanip>
#include <iostream>
#include <set>
#include <string>
#include <vector>

namespace a_star
{

using bitsim::real_t;
using namespace bitsim::planning;

const real_t WIDTH = 8.0;         // [m] of the warehouse, along x
const real_t HEIGHT = 5.0;        // [m] along y
const real_t CELL = 0.1;          // [m]
const real_t ROBOT_RADIUS = 0.25; // [m]

// The shelves and the wall, as rectangles: x_min, y_min, x_max, y_max [m]
struct Box
{
    real_t x0, y0, x1, y1;
};
const std::vector<Box> OBSTACLES{
    {1.5, 1.0, 2.0, 4.0}, // a shelf
    {3.5, 0.0, 3.7, 3.0}, // a wall, with a door above it
    {3.5, 3.8, 3.7, 5.0}, // the rest of the wall
    {5.0, 1.5, 6.5, 2.0}, // a shelf
    {5.5, 3.0, 6.0, 4.5}, // a shelf
};

const std::array<real_t, 2> START{0.5, 0.5};
const std::array<real_t, 2> GOAL{7.5, 4.5};

// The grid the planner searches: the obstacles grown by the robot's radius, and the walls of
// the warehouse as well
OccupancyGrid planning_grid(real_t margin)
{
    OccupancyGrid grid(static_cast<bitsim::uint_t>(WIDTH / CELL),
                       static_cast<bitsim::uint_t>(HEIGHT / CELL), CELL);
    for (const auto &b : OBSTACLES)
    {
        grid.add_rectangle(b.x0 - margin, b.y0 - margin, b.x1 + margin, b.y1 + margin);
    }
    grid.add_rectangle(0.0, 0.0, WIDTH, margin);
    grid.add_rectangle(0.0, HEIGHT - margin, WIDTH, HEIGHT);
    grid.add_rectangle(0.0, 0.0, margin, HEIGHT);
    grid.add_rectangle(WIDTH - margin, 0.0, WIDTH, HEIGHT);
    return grid;
}

// The warehouse from above, two cells a character: # an obstacle, : where the robot's centre
// cannot go, * the path, S and G the start and the goal
void draw(const OccupancyGrid &obstacles, const OccupancyGrid &grown,
          const std::vector<GridCell> &path)
{
    std::set<std::pair<int, int>> on_path;
    for (const auto &c : path)
    {
        on_path.insert({static_cast<int>(c.col / 2), static_cast<int>(c.row / 2)});
    }
    const GridCell s = grown.to_cell(START[0], START[1]);
    const GridCell g = grown.to_cell(GOAL[0], GOAL[1]);
    const int cols = static_cast<int>(grown.n_cols() / 2);
    const int rows = static_cast<int>(grown.n_rows() / 2);
    for (int r = rows - 1; r >= 0; --r)
    {
        std::cout << "  ";
        for (int c = 0; c < cols; ++c)
        {
            const GridCell cell{2 * c, 2 * r};
            char ch = ' ';
            if (!obstacles.is_free(cell))
            {
                ch = '#';
            }
            else if (!grown.is_free(cell))
            {
                ch = ':';
            }
            if (on_path.count({c, r}))
            {
                ch = '*';
            }
            if (c == s.col / 2 && r == s.row / 2)
            {
                ch = 'S';
            }
            if (c == g.col / 2 && r == g.row / 2)
            {
                ch = 'G';
            }
            std::cout << ch;
        }
        std::cout << "\n";
    }
}

void report(const std::string &name, const SearchResult<GridCell> &result)
{
    std::cout << "  " << std::left << std::setw(26) << name << std::right;
    if (!result.found)
    {
        std::cout << "no path\n";
        return;
    }
    std::cout << std::fixed << std::setprecision(2) << std::setw(8) << result.cost << " m"
              << std::setw(10) << result.n_expanded << " cells expanded\n";
}

void run()
{
    const OccupancyGrid obstacles = planning_grid(0.0);
    const OccupancyGrid grid = planning_grid(ROBOT_RADIUS);
    const GridCell start = grid.to_cell(START[0], START[1]);
    const GridCell goal = grid.to_cell(GOAL[0], GOAL[1]);

    bitsim::utils::io::print_title("1. The warehouse and the shortest path");
    std::cout << "  " << grid.n_cols() << " x " << grid.n_rows() << " cells of " << CELL << " m, "
              << grid.n_occupied() << " of them too close to an obstacle\n\n";
    const auto best = a_star_grid(grid, start, goal);
    draw(obstacles, grid, best.path);

    bitsim::utils::io::print_title("2. How the searches compare");
    std::cout << "  search                        length   expanded\n";
    report("A*", best);
    report("Dijkstra (weight 0)", a_star_grid(grid, start, goal, true, 0.0));
    report("weighted A* (weight 3)", a_star_grid(grid, start, goal, true, 3.0));
    report("A*, moves along the axes", a_star_grid(grid, start, goal, false));

    bitsim::utils::io::print_title("3. A robot that is too wide for the door");
    const OccupancyGrid wide = planning_grid(0.45);
    report("A*, robot radius 0.45 m", a_star_grid(wide, start, goal));
}

} // namespace a_star

int main()
{
    a_star::run();
    return 0;
}

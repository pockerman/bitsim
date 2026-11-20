#include "bitsim/planning/a_star_search.h"

#include <cmath>

namespace bitsim
{
namespace planning
{

SearchResult<GridCell> a_star_grid(const OccupancyGrid &grid, const GridCell &start,
                                   const GridCell &goal, bool diagonal, real_t weight)
{
    if (!grid.is_free(start) || !grid.is_free(goal))
    {
        return {};
    }

    const auto distance = [&grid](const GridCell &a, const GridCell &b)
    {
        return grid.cell_size() *
               std::hypot(static_cast<real_t>(a.col - b.col), static_cast<real_t>(a.row - b.row));
    };
    return a_star(
        start, goal, [&grid, diagonal](const GridCell &c) { return grid.neighbours(c, diagonal); },
        distance, distance, weight);
}

} // namespace planning
} // namespace bitsim

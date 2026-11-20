#include "bitsim/planning/occupancy_grid.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace bitsim
{
namespace planning
{

OccupancyGrid::OccupancyGrid(uint_t n_cols, uint_t n_rows, real_t cell_size, real_t x0, real_t y0)
    : n_cols_(n_cols), n_rows_(n_rows), cell_size_(cell_size), x0_(x0), y0_(y0),
      occupied_(n_cols * n_rows, false)
{
    if (n_cols == 0 || n_rows == 0 || cell_size <= 0.0)
    {
        throw std::invalid_argument("OccupancyGrid needs cells, and a positive cell size");
    }
}

bool OccupancyGrid::in_bounds(const GridCell &c) const noexcept
{
    return c.col >= 0 && c.row >= 0 && static_cast<uint_t>(c.col) < n_cols_ &&
           static_cast<uint_t>(c.row) < n_rows_;
}

bool OccupancyGrid::is_free(const GridCell &c) const noexcept
{
    return in_bounds(c) && !occupied_[index_(c)];
}

void OccupancyGrid::set_occupied(const GridCell &c, bool occupied)
{
    if (!in_bounds(c))
    {
        throw std::out_of_range("OccupancyGrid: the cell is not in the grid");
    }
    occupied_[index_(c)] = occupied;
}

void OccupancyGrid::add_rectangle(real_t x_min, real_t y_min, real_t x_max, real_t y_max)
{
    for (uint_t row = 0; row < n_rows_; ++row)
    {
        for (uint_t col = 0; col < n_cols_; ++col)
        {
            const GridCell c{static_cast<int_t>(col), static_cast<int_t>(row)};
            const auto p = to_world(c);
            if (p[0] >= x_min && p[0] <= x_max && p[1] >= y_min && p[1] <= y_max)
            {
                occupied_[index_(c)] = true;
            }
        }
    }
}

void OccupancyGrid::add_circle(real_t x, real_t y, real_t radius)
{
    for (uint_t row = 0; row < n_rows_; ++row)
    {
        for (uint_t col = 0; col < n_cols_; ++col)
        {
            const GridCell c{static_cast<int_t>(col), static_cast<int_t>(row)};
            const auto p = to_world(c);
            if (std::hypot(p[0] - x, p[1] - y) <= radius)
            {
                occupied_[index_(c)] = true;
            }
        }
    }
}

GridCell OccupancyGrid::to_cell(real_t x, real_t y) const noexcept
{
    return {static_cast<int_t>(std::floor((x - x0_) / cell_size_)),
            static_cast<int_t>(std::floor((y - y0_) / cell_size_))};
}

std::array<real_t, 2> OccupancyGrid::to_world(const GridCell &c) const noexcept
{
    return {x0_ + (c.col + 0.5) * cell_size_, y0_ + (c.row + 0.5) * cell_size_};
}

std::vector<GridCell> OccupancyGrid::neighbours(const GridCell &c, bool diagonal) const
{
    std::vector<GridCell> result;
    for (int_t dr = -1; dr <= 1; ++dr)
    {
        for (int_t dc = -1; dc <= 1; ++dc)
        {
            if (dr == 0 && dc == 0)
            {
                continue;
            }
            const bool is_diagonal = dr != 0 && dc != 0;
            if (is_diagonal && !diagonal)
            {
                continue;
            }
            const GridCell n{c.col + dc, c.row + dr};
            if (!is_free(n))
            {
                continue;
            }
            if (is_diagonal && (!is_free({c.col + dc, c.row}) || !is_free({c.col, c.row + dr})))
            {
                continue;
            }
            result.push_back(n);
        }
    }
    return result;
}

uint_t OccupancyGrid::n_occupied() const
{
    return static_cast<uint_t>(std::count(occupied_.begin(), occupied_.end(), true));
}

std::size_t OccupancyGrid::index_(const GridCell &c) const noexcept
{
    return static_cast<std::size_t>(c.row) * n_cols_ + static_cast<std::size_t>(c.col);
}

} // namespace planning
} // namespace bitsim

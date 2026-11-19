#ifndef BITRL_PLANNING_OCCUPANCY_GRID_H
#define BITRL_PLANNING_OCCUPANCY_GRID_H

#include "bitrl/bitrl_types.h"

#include <array>
#include <cstddef>
#include <functional>
#include <vector>

namespace bitrl
{
namespace planning
{

///
/// @ingroup bitrl_planning
/// @brief A cell of an OccupancyGrid: its column, along x, and row, along y
///
struct GridCell
{
    int_t col{0};
    int_t row{0};

    bool operator==(const GridCell &other) const noexcept
    {
        return col == other.col && row == other.row;
    }
    bool operator!=(const GridCell &other) const noexcept { return !(*this == other); }
};

/**
 * @class OccupancyGrid
 * @ingroup bitrl_planning
 * @brief A rectangle of the plane cut into square cells, each free or occupied.
 *
 * Cell (0, 0) has its lower left corner at the origin given to the constructor; columns grow
 * with x and rows with y. Obstacles are added as rectangles and circles: every cell whose
 * centre is inside one is occupied. To keep a robot of radius r clear of the obstacles, grow
 * them by r when adding them.
 */
class OccupancyGrid
{
  public:
    ///
    /// @param n_cols The number of columns, along x
    /// @param n_rows The number of rows, along y
    /// @param cell_size The side of a cell [m]
    /// @param x0 The x of the lower left corner of the grid [m]
    /// @param y0 The y of the lower left corner of the grid [m]
    ///
    OccupancyGrid(uint_t n_cols, uint_t n_rows, real_t cell_size, real_t x0 = 0.0, real_t y0 = 0.0);

    uint_t n_cols() const noexcept { return n_cols_; }
    uint_t n_rows() const noexcept { return n_rows_; }
    real_t cell_size() const noexcept { return cell_size_; }

    bool in_bounds(const GridCell &c) const noexcept;

    ///
    /// @brief True if the cell is in the grid and not occupied
    ///
    bool is_free(const GridCell &c) const noexcept;

    void set_occupied(const GridCell &c, bool occupied = true);

    ///
    /// @brief Occupy the cells whose centres are in the rectangle [x_min, x_max] x [y_min, y_max]
    ///
    void add_rectangle(real_t x_min, real_t y_min, real_t x_max, real_t y_max);

    ///
    /// @brief Occupy the cells whose centres are within radius of (x, y)
    ///
    void add_circle(real_t x, real_t y, real_t radius);

    ///
    /// @brief The cell that the point (x, y) is in. It may be out of bounds
    ///
    GridCell to_cell(real_t x, real_t y) const noexcept;

    ///
    /// @brief The centre of a cell [m]
    ///
    std::array<real_t, 2> to_world(const GridCell &c) const noexcept;

    ///
    /// @brief The free cells next to c: the four that share a side, and with diagonal the four
    /// that share a corner as well. A diagonal move is left out when either cell it passes
    /// between is occupied, so a path never cuts the corner of an obstacle
    ///
    std::vector<GridCell> neighbours(const GridCell &c, bool diagonal = true) const;

    ///
    /// @brief The number of occupied cells
    ///
    uint_t n_occupied() const;

  private:
    std::size_t index_(const GridCell &c) const noexcept;

    uint_t n_cols_;
    uint_t n_rows_;
    real_t cell_size_;
    real_t x0_;
    real_t y0_;
    std::vector<bool> occupied_;
};

} // namespace planning
} // namespace bitrl

/// \cond
// So that a GridCell can be the key of an std::unordered_map, as in a_star
template <> struct std::hash<bitrl::planning::GridCell>
{
    std::size_t operator()(const bitrl::planning::GridCell &c) const noexcept
    {
        return std::hash<long long>()((static_cast<long long>(c.col) << 32) ^
                                      static_cast<long long>(c.row & 0xffffffff));
    }
};
/// \endcond

#endif // BITRL_PLANNING_OCCUPANCY_GRID_H

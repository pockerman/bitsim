#ifndef BITRL_TRAJECTORY_ROUNDED_RECTANGLE_H
#define BITRL_TRAJECTORY_ROUNDED_RECTANGLE_H

#include "bitrl/bitrl_types.h"

namespace bitrl
{
namespace trajectory
{

/**
 * @class RoundedRectangle
 * @brief A closed track in the plane: a rectangle centred on the origin with its sides along x
 * and y, and its corners rounded with quarter circles.
 *
 * The track is the boundary of the shape, the line a robot follows. With a corner radius of
 * half the width it is a stadium; with a radius of zero, a rectangle.
 */
class RoundedRectangle
{
  public:
    ///
    /// @param half_length Half the length along x, to the line [m]
    /// @param half_width Half the width along y [m]
    /// @param corner_radius The radius of the corners [m], at most the smaller half side
    ///
    RoundedRectangle(real_t half_length, real_t half_width, real_t corner_radius);

    real_t half_length() const noexcept { return half_length_; }
    real_t half_width() const noexcept { return half_width_; }
    real_t corner_radius() const noexcept { return corner_radius_; }

    ///
    /// @brief The distance from (x, y) to the track [m], from inside or outside
    ///
    real_t distance(real_t x, real_t y) const;

    ///
    /// @brief The length of the track [m]
    ///
    real_t perimeter() const;

  private:
    real_t half_length_;
    real_t half_width_;
    real_t corner_radius_;
};

} // namespace trajectory
} // namespace bitrl

#endif // BITRL_TRAJECTORY_ROUNDED_RECTANGLE_H

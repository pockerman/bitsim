#include "bitrl/trajectory/rounded_rectangle.h"
#include "bitrl/bitrl_consts.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace bitrl
{
namespace trajectory
{

RoundedRectangle::RoundedRectangle(real_t half_length, real_t half_width, real_t corner_radius)
    : half_length_(half_length), half_width_(half_width), corner_radius_(corner_radius)
{
    if (half_length <= 0.0 || half_width <= 0.0 || corner_radius < 0.0 ||
        corner_radius > std::min(half_length, half_width))
    {
        throw std::invalid_argument("RoundedRectangle: the sides must be positive and the corner "
                                    "radius at most the smaller half side");
    }
}

real_t RoundedRectangle::distance(real_t x, real_t y) const
{
    // The signed distance to the boundary of the shape: shrink the rectangle by the radius,
    // measure the distance to it, and take the radius off again
    const real_t qx = std::abs(x) - (half_length_ - corner_radius_);
    const real_t qy = std::abs(y) - (half_width_ - corner_radius_);
    const real_t outside = std::hypot(std::max(qx, 0.0), std::max(qy, 0.0));
    const real_t inside = std::min(std::max(qx, qy), 0.0);
    return std::abs(outside + inside - corner_radius_);
}

real_t RoundedRectangle::perimeter() const
{
    return 4.0 * (half_length_ - corner_radius_) + 4.0 * (half_width_ - corner_radius_) +
           2.0 * consts::maths::PI * corner_radius_;
}

} // namespace trajectory
} // namespace bitrl

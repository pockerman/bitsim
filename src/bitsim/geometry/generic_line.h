#ifndef BITSIM_GEOMETRY_GENERIC_LINE_H
#define BITSIM_GEOMETRY_GENERIC_LINE_H

#include "bitsim/bitsim_consts.h"
#include "bitsim/bitsim_types.h"
#include "bitsim/geometry/geom_point.h"
#include "bitsim/maths/math_utils.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace bitsim
{
namespace geometry
{

///
/// \brief class GenericLine. Represents a generic line
/// with vertex VertexType
/// The equation of a linein l=Ax + By + C
///
template <int dim> class GenericLine
{
  public:
    static const int dimension = dim;

    typedef GeomPoint<dim> vertex_type;

    ///
    /// \brief Constructor
    ///
    GenericLine();

    ///
    /// \brief Constructor
    ///
    GenericLine(const vertex_type &p1, const vertex_type &p2);

    ///
    /// \brief Constructor
    ///
    GenericLine(const vertex_type &p1, const vertex_type &p2, uint_t id);

    ///
    /// \brief Returns the v-th vertex of the segment
    ///
    const vertex_type &get_vertex(uint_t v) const;

    ///
    /// \brief Returns the v-th vertex of the segment
    ///
    vertex_type &get_vertex(uint_t v);

    ///
    /// \brief Returns true iff id_ != rlenvscpp::consts::INVALID_ID
    ///
    bool has_valid_id() const noexcept { return id_ != bitsim::consts::INVALID_ID; }

    ///
    /// \brief Returns the id of the line
    ///
    uint_t get_id() const noexcept { return id_; }

    ///
    /// \brief Set the id of the line
    ///
    void set_id(uint_t id) noexcept { id_ = id; }

    ///
    /// \brief The slope of the line; infinite for a vertical line
    ///
    real_t slope() const;

    ///
    /// \brief Returns the constant factor
    ///
    real_t factor() const;

    ///
    /// \brief The perpendicular distance of the point from the infinite line through the two
    /// vertices; the distance to the first vertex if the two are the same
    ///
    real_t distance(const vertex_type &n) const;

    ///
    /// \brief The distance of the point from the segment between the two vertices
    ///
    real_t segment_distance(const vertex_type &n) const;

    ///
    /// \brief Calculate the length of the line
    ///
    real_t length() const { return end_.distance(start_); }

  private:
    uint_t id_;
    vertex_type start_;
    vertex_type end_;
};

template <int dim>
GenericLine<dim>::GenericLine() : id_(bitsim::consts::INVALID_ID), start_(), end_()
{
}

template <int dim>
GenericLine<dim>::GenericLine(const vertex_type &p1, const vertex_type &p2, uint_t id)
    : id_(id), start_(p1), end_(p2)
{
}

template <int dim>
GenericLine<dim>::GenericLine(const vertex_type &p1, const vertex_type &p2)
    : GenericLine<dim>(p1, p2, bitsim::consts::INVALID_ID)
{
}

template <int dim>
const typename GenericLine<dim>::vertex_type &GenericLine<dim>::get_vertex(uint_t v) const
{

    if (v == 0)
    {
        return start_;
    }
    else if (v == 1)
    {
        return end_;
    }

    throw std::logic_error("Invalid vertex index. Index not in [0,1]");
}

template <int dim> typename GenericLine<dim>::vertex_type &GenericLine<dim>::get_vertex(uint_t v)
{

    if (v == 0)
    {
        return start_;
    }
    else if (v == 1)
    {
        return end_;
    }

    throw std::logic_error("Invalid vertex index. Index not in [0,1]");
}

template <int dim> real_t GenericLine<dim>::slope() const
{
    return (end_[1] - start_[1]) / (end_[0] - start_[0]);
}

template <int dim> real_t GenericLine<dim>::factor() const
{
    auto slope_ = slope();
    return start_[1] - slope_ * start_[0];
}

// The specialisations are inline: this header can be included in more than one source file
template <> inline real_t GenericLine<2>::distance(const GenericLine<2>::vertex_type &n) const
{
    // |d x (n - start)| / |d| with d the direction of the line, which, unlike the slope, works for
    // a vertical line too
    const real_t dx = end_[0] - start_[0];
    const real_t dy = end_[1] - start_[1];
    const real_t length = std::hypot(dx, dy);
    if (length == 0.0)
    {
        return std::hypot(n[0] - start_[0], n[1] - start_[1]);
    }
    return std::fabs(dx * (n[1] - start_[1]) - dy * (n[0] - start_[0])) / length;
}

template <>
inline real_t GenericLine<2>::segment_distance(const GenericLine<2>::vertex_type &n) const
{
    // The nearest point of the segment is start + t d, with t the projection clamped to [0, 1]
    const real_t dx = end_[0] - start_[0];
    const real_t dy = end_[1] - start_[1];
    const real_t length2 = dx * dx + dy * dy;
    const real_t t =
        length2 == 0.0
            ? 0.0
            : std::clamp(((n[0] - start_[0]) * dx + (n[1] - start_[1]) * dy) / length2, 0.0, 1.0);
    return std::hypot(n[0] - (start_[0] + t * dx), n[1] - (start_[1] + t * dy));
}

} // namespace geometry
} // namespace bitsim
#endif // BITSIM_GEOMETRY_GENERIC_LINE_H

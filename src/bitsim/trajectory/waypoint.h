#ifndef BITSIM_TRAJECTORY_WAYPOINT_H
#define BITSIM_TRAJECTORY_WAYPOINT_H

#include "bitsim/bitsim_consts.h"
#include "bitsim/bitsim_types.h"
#include "bitsim/geometry/geom_point.h"
namespace bitsim
{
namespace trajectory
{

using bitsim::geometry::GeomPoint;

///
/// \brief Helper class to represent a waypoint
///
template <int dim, typename Data> class WayPoint : public GeomPoint<dim>
{
  public:
    typedef Data data_type;

    ///
    /// constructor
    ///
    WayPoint();

    ///
    /// constructor
    ///
    WayPoint(const GeomPoint<dim> &p, uint_t id, const data_type &data_ = data_type());

    ///
    /// \brief Return the id of the point
    ///
    uint_t get_id() const { return id_; }

    ///
    /// \brief Writable reference to the data
    ///
    data_type &get_data() { return data_; }

    ///
    /// \brief Read reference to the data
    ///
    const data_type &get_data() const { return data_; }

    ///
    /// \brief Returns true if the waypoint is active
    ///
    bool is_active() const { return is_active_point_; }

  private:
    uint_t id_;
    data_type data_;
    bool is_active_point_;
};

template <int dim, typename Data>
WayPoint<dim, Data>::WayPoint(const GeomPoint<dim> &p, uint_t id,
                              const typename WayPoint<dim, Data>::data_type &data)
    : GeomPoint<dim>(p), id_(id), data_(data), is_active_point_(true)
{
}

template <int dim, typename Data>
WayPoint<dim, Data>::WayPoint() : WayPoint(GeomPoint<dim>(), bitsim::consts::INVALID_ID)
{
}
} // namespace trajectory
} // namespace bitsim

#endif // BITSIM_TRAJECTORY_WAYPOINT_H

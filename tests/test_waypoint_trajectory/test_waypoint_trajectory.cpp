#include "bitsim/bitsim_types.h"
#include "bitsim/geometry/geom_point.h"
#include "bitsim/trajectory/line_segment_link.h"
#include "bitsim/trajectory/waypoint.h"
#include "bitsim/trajectory/waypoint_trajectory.h"

#include <gtest/gtest.h>

#include <string>

namespace
{
using bitsim::real_t;
using bitsim::uint_t;
using bitsim::geometry::GeomPoint;
using Link = bitsim::trajectory::LineSegmentLink<2, std::string, real_t>;
using Point = Link::w_point_type;
using Trajectory = bitsim::trajectory::WaypointTrajectory<Link>;

Point waypoint(real_t x, real_t y, uint_t id) { return Point(GeomPoint<2>({x, y}), id, "wp"); }
} // namespace

TEST(TestLineSegmentLink, DistanceIsToTheSegmentNotTheLine)
{
    const Link link(waypoint(0.0, 0.0, 0), waypoint(2.0, 0.0, 1), 7, 0.5);
    EXPECT_EQ(link.get_id(), 7u);
    EXPECT_TRUE(link.is_active());
    EXPECT_DOUBLE_EQ(link.get_data(), 0.5);
    EXPECT_DOUBLE_EQ(link.length(), 2.0);

    EXPECT_DOUBLE_EQ(link.distance(GeomPoint<2>({1.0, 0.5})), 0.5);  // beside it
    EXPECT_DOUBLE_EQ(link.distance(GeomPoint<2>({3.0, 0.0})), 1.0);  // beyond its end
    EXPECT_DOUBLE_EQ(link.distance(GeomPoint<2>({-3.0, 4.0})), 5.0); // before its start
}

TEST(TestLineSegmentLink, VerticalSegment)
{
    // A vertical line has no finite slope; the distance does not need one
    const Link link(waypoint(1.0, 0.0, 0), waypoint(1.0, 2.0, 1), 0);
    EXPECT_DOUBLE_EQ(link.distance(GeomPoint<2>({3.0, 1.0})), 2.0);
    EXPECT_DOUBLE_EQ(link.distance(GeomPoint<2>({1.0, 5.0})), 3.0);
}

TEST(TestWaypointTrajectory, FindsTheNearestLink)
{
    // An L: along x from (0, 0) to (4, 0), then up to (4, 3)
    Trajectory trajectory;
    EXPECT_TRUE(trajectory.empty());
    trajectory.push(Link(waypoint(0.0, 0.0, 0), waypoint(4.0, 0.0, 1), 0));
    trajectory.push(Link(waypoint(4.0, 0.0, 1), waypoint(4.0, 3.0, 2), 1));
    EXPECT_EQ(trajectory.size(), 2u);

    const auto [d1, near1] = trajectory.distance(Point(GeomPoint<2>({1.0, 0.3}), 9));
    EXPECT_NEAR(d1, 0.3, 1.0e-12);
    EXPECT_EQ(near1.get_id(), 0u);

    const auto [d2, near2] = trajectory.distance(Point(GeomPoint<2>({4.5, 2.0}), 9));
    EXPECT_NEAR(d2, 0.5, 1.0e-12);
    EXPECT_EQ(near2.get_id(), 1u);
}

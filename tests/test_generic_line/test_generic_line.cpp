#include "bitrl/geometry/geom_point.h"
#include "bitrl/bitrl_types.h"
#include "bitrl/geometry/generic_line.h"

#include <gtest/gtest.h>


namespace{

using bitrl::uint_t;
using bitrl::real_t;
using bitrl::geometry::GeomPoint;
using bitrl::geometry::GenericLine;

}

TEST(TestGenericLine, slope1) {

    GeomPoint<2> p0({0.0, 0.0});
	GeomPoint<2> p1({1.0, 0.0});

	GenericLine<2> line(p0, p1);
	auto slope = line.slope();

    EXPECT_DOUBLE_EQ(slope, 0.0);
    
}

TEST(TestGenericLine, slope2) {
	
	// assume the line y = -3/4 x + 6/4
	// x = 0 => y = 6/4
	// x = 4 => y = - 6/4

    GeomPoint<2> p0({0.0, 6.0/4.0});
	GeomPoint<2> p1({4.0, -6.0 /4.0});
	GenericLine<2> line(p0, p1);
	auto slope = line.slope();

    EXPECT_DOUBLE_EQ(slope, -3.0/4.0);
    
}

TEST(TestGenericLine, factor1) {

   // assume the line y = -3/4 x + 6/4
	// x = 0 => y = 6/4
	// x = 4 => y = - 6/4

    GeomPoint<2> p0({0.0, 6.0/4.0});
	GeomPoint<2> p1({4.0, -6.0 / 4.0});
	GenericLine<2> line(p0, p1);
	auto factor = line.factor();

    EXPECT_DOUBLE_EQ(factor, 6.0/4.0);
    
}

TEST(TestGenericLine, factor2) {

    GeomPoint<2> p0({0.0, 0.0});
	GeomPoint<2> p1({1.0, 0.0});
	GenericLine<2> line(p0, p1);
	auto factor = line.factor();

    EXPECT_DOUBLE_EQ(factor, 0.0);
    
}

TEST(TestGenericLine, distance1) {

    GeomPoint<2> p0({0.0, 0.0});
	GeomPoint<2> p1({1.0, 0.0});
	GenericLine<2> line(p0, p1);
	
	GeomPoint<2> p({0.5, 0.5});
	auto dist = line.distance(p);

    EXPECT_DOUBLE_EQ(dist, 0.5);
    
}


TEST(TestGenericLine, distance2) {
	
	// assume the line y = -3/4 x + 6/4
	// x = 0 => y = 6/4
	// x = 4 => y = - 6/4

    GeomPoint<2> p0({0.0, 6.0/4.0});
	GeomPoint<2> p1({4.0, -6.0 / 4.0});

	GenericLine<2> line(p0, p1);
	
	GeomPoint<2> p({0.0, 0.0});
	auto dist = line.distance(p);

    EXPECT_DOUBLE_EQ(dist, 6.0 / 5.0);
    
}














TEST(TestGenericLine, distanceVerticalLine)
{
    // A vertical line has no finite slope; the distance does not need one
    GenericLine<2> line(GeomPoint<2>({1.0, 0.0}), GeomPoint<2>({1.0, 2.0}));
    EXPECT_DOUBLE_EQ(line.distance(GeomPoint<2>({3.0, 7.0})), 2.0);
    // The segment ends at y = 2
    EXPECT_DOUBLE_EQ(line.segment_distance(GeomPoint<2>({1.0, 7.0})), 5.0);
}

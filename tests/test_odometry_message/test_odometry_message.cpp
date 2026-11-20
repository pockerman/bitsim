#include "bitsim/sensors/messages/odometry.h"

#include <gtest/gtest.h>

#include <chrono>

using bitsim::sensors::OdometryMessage;

TEST(TestOdometryMessage, ParsesAPose)
{
    const auto m = OdometryMessage::parse(R"({"x": 1.25, "y": -0.4, "yaw": 0.52})");
    ASSERT_TRUE(m.has_value());
    EXPECT_DOUBLE_EQ(m->x, 1.25);
    EXPECT_DOUBLE_EQ(m->y, -0.4);
    EXPECT_DOUBLE_EQ(m->yaw, 0.52);
    // Without a timestamp the message counts from when it arrived
    EXPECT_EQ(m->source_timestamp, m->received_timestamp);
}

TEST(TestOdometryMessage, ReadsTheTimestamp)
{
    const auto m =
        OdometryMessage::parse(R"({"x": 0, "y": 0, "yaw": 0, "timestamp": "2026-11-18 10:15:02"})");
    ASSERT_TRUE(m.has_value());
    const auto expected = OdometryMessage::parse_utc_timestamp("2026-11-18 10:15:02");
    EXPECT_EQ(m->source_timestamp, expected);
}

TEST(TestOdometryMessage, RejectsWhatIsNotAPose)
{
    EXPECT_FALSE(OdometryMessage::parse("not json").has_value());
    EXPECT_FALSE(OdometryMessage::parse("[1, 2, 3]").has_value());
    EXPECT_FALSE(OdometryMessage::parse(R"({"x": 1, "y": 2})").has_value());
    EXPECT_FALSE(OdometryMessage::parse(R"({"x": "1", "y": 2, "yaw": 0})").has_value());
    EXPECT_FALSE(
        OdometryMessage::parse(R"({"x": 1, "y": 2, "yaw": 0, "timestamp": 5})").has_value());
    EXPECT_FALSE(
        OdometryMessage::parse(R"({"x": 1, "y": 2, "yaw": 0, "timestamp": "soon"})").has_value());
}

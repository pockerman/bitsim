// The pieces a line following robot is made of: the track, the infrared line follower and the
// heading from a gyroscope

#include "bitsim/bitsim_consts.h"
#include "bitsim/bitsim_types.h"
#include "bitsim/control/infrared_line_follower.h"
#include "bitsim/estimation/gyro_heading.h"
#include "bitsim/sensors/backends/infrared_backend_base.h"
#include "bitsim/sensors/infrared_sensor.h"
#include "bitsim/trajectory/rounded_rectangle.h"

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>

namespace
{
using bitsim::real_t;
using bitsim::control::InfraredLineFollower;
using bitsim::estimation::GyroHeading;
using bitsim::sensors::InfraredSensor;
using bitsim::trajectory::RoundedRectangle;

const real_t PI = bitsim::consts::maths::PI;

// Three elements whose readings the test sets
class FakeInfrared : public bitsim::sensors::backends::InfraredBackendBase
{
  public:
    FakeInfrared() : InfraredBackendBase("FAKE")
    {
        set_elements({{0.2, 0.05, 0.0}, {0.2, 0.0, 0.0}, {0.2, -0.05, 0.0}});
    }

    std::vector<real_t> read_values() override { return values; }

    std::vector<real_t> values{1.0, 1.0, 1.0};
};

struct Line
{
    std::shared_ptr<FakeInfrared> backend = std::make_shared<FakeInfrared>();
    InfraredSensor ir{backend};

    Line() { ir.init(); }

    // '#' for an element over the line
    void see(const std::string &pattern)
    {
        for (std::size_t i = 0; i < pattern.size(); ++i)
        {
            backend->values[i] = pattern[i] == '#' ? 0.1 : 0.9;
        }
        ir.read_values();
    }
};
} // namespace

TEST(TestRoundedRectangle, DistanceToTheLine)
{
    const RoundedRectangle track(2.2, 1.4, 0.7);
    EXPECT_NEAR(track.distance(0.0, -1.4), 0.0, 1.0e-12); // on the bottom side
    EXPECT_NEAR(track.distance(2.2, 0.0), 0.0, 1.0e-12);  // on the right side
    EXPECT_NEAR(track.distance(0.0, 0.0), 1.4, 1.0e-12);  // the centre, from the long sides
    EXPECT_NEAR(track.distance(0.0, -1.6), 0.2, 1.0e-12); // outside
    // the corner's centre is at (1.5, 0.7): a point on its arc
    EXPECT_NEAR(track.distance(1.5 + 0.7 * std::cos(-PI / 4), 0.7 + 0.7 * std::sin(-PI / 4) - 1.4),
                0.0, 1.0e-12);
    EXPECT_NEAR(track.perimeter(), 4 * 1.5 + 4 * 0.7 + 2 * PI * 0.7, 1.0e-12);
    EXPECT_THROW(RoundedRectangle(1.0, 0.5, 0.6), std::invalid_argument);
}

TEST(TestInfraredLineFollower, TurnsTowardsTheLine)
{
    Line line;
    EXPECT_EQ(line.ir.pattern(), "...");

    InfraredLineFollower follower;
    const auto &p = follower.parameters();

    line.see(".#.");
    EXPECT_EQ(line.ir.pattern(), ".#.");
    auto command = follower.command(line.ir, 0.01);
    EXPECT_DOUBLE_EQ(command.v, p.cruise_speed);
    EXPECT_DOUBLE_EQ(command.omega, 0.0);

    line.see("#..");
    command = follower.command(line.ir, 0.01);
    EXPECT_DOUBLE_EQ(command.v, 0.5 * p.cruise_speed);
    EXPECT_DOUBLE_EQ(command.omega, p.line_turn_rate); // left, counterclockwise

    line.see("..#");
    command = follower.command(line.ir, 0.01);
    EXPECT_DOUBLE_EQ(command.omega, -p.line_turn_rate);
}

TEST(TestInfraredLineFollower, SearchesWhereItSawTheLineLastAndGivesUp)
{
    Line line;
    InfraredLineFollower follower({0.3, 1.6, 0.8, 1.0});

    line.see("#..");
    follower.command(line.ir, 0.01);

    line.see("...");
    const auto command = follower.command(line.ir, 0.5);
    EXPECT_DOUBLE_EQ(command.v, 0.0);
    EXPECT_DOUBLE_EQ(command.omega, 0.8);
    EXPECT_TRUE(follower.is_lost());
    EXPECT_FALSE(follower.has_given_up());

    follower.command(line.ir, 0.6);
    EXPECT_TRUE(follower.has_given_up());

    line.see(".#.");
    follower.command(line.ir, 0.01);
    EXPECT_FALSE(follower.is_lost());
}

TEST(TestGyroHeading, CalibrationTakesTheBiasOff)
{
    const real_t bias = 0.003;
    const real_t rate = 0.2;
    const real_t dt = 0.01;

    GyroHeading calibrated(0.0, 1.0);
    GyroHeading uncalibrated(0.0, 0.0);
    EXPECT_TRUE(calibrated.is_calibrating());
    EXPECT_FALSE(uncalibrated.is_calibrating());

    // a second standing still, then 5 s turning
    for (int k = 0; k < 100; ++k)
    {
        calibrated.update(bias, dt);
        uncalibrated.update(bias, dt);
    }
    EXPECT_FALSE(calibrated.is_calibrating());
    EXPECT_NEAR(calibrated.bias(), bias, 1.0e-12);
    EXPECT_DOUBLE_EQ(calibrated.heading(), 0.0);

    for (int k = 0; k < 500; ++k)
    {
        calibrated.update(rate + bias, dt);
        uncalibrated.update(rate + bias, dt);
    }
    EXPECT_NEAR(calibrated.heading(), 1.0, 1.0e-9);
    EXPECT_NEAR(uncalibrated.heading(), 1.0 + 6.0 * bias, 1.0e-9);
}

TEST(TestGyroHeading, HeadingStaysWithinHalfATurn)
{
    GyroHeading heading(3.0, 0.0);
    heading.update(1.0, 0.5);
    EXPECT_NEAR(heading.heading(), 3.5 - 2.0 * PI, 1.0e-12);
    EXPECT_THROW(GyroHeading(0.0, -1.0), std::invalid_argument);
}

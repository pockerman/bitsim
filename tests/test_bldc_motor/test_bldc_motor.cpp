#include "bitsim/actuators/bldc_motor.h"
#include "bitsim/bitsim_consts.h"
#include "bitsim/bitsim_types.h"
#include "bitsim/control/pi_controller.h"

#include <gtest/gtest.h>

#include <set>
#include <stdexcept>

namespace
{
using bitsim::real_t;
using namespace bitsim::actuators;
using bitsim::control::PIController;

const real_t PI = bitsim::consts::maths::PI;
const real_t DEG = PI / 180.0;

// The steady speed of the DC-equivalent model with no load: Kt V = (R b + Ke Kt) w
real_t no_load_speed(const BLDCMotorParameters &m, real_t V)
{
    return m.Kt() * V / (m.R * m.b + m.Ke * m.Kt());
}
} // namespace

TEST(TestBLDCMotor, DCEquivalentReachesItsNoLoadSpeed)
{
    const BLDCMotorParameters m;
    DCEquivalentState s;
    const real_t dt = 1.0e-6;
    for (int k = 0; k < 2000000; ++k)
    {
        s = dc_equivalent_step(m, s, 12.0, 0.0, dt);
    }
    EXPECT_NEAR(s.w, no_load_speed(m, 12.0), 1.0e-3 * no_load_speed(m, 12.0));
}

TEST(TestBLDCMotor, HallSensorsGiveSixSectors)
{
    std::set<int> codes;
    for (int k = 0; k < 360; ++k)
    {
        const int code = BLDCMotor::hall_code((k + 0.5) * DEG);
        EXPECT_NE(code, 0);
        EXPECT_NE(code, 7);
        codes.insert(code);

        const PhasePair pair = BLDCMotor::commutation(code);
        EXPECT_NE(pair.high, pair.low);
    }
    EXPECT_EQ(codes.size(), 6u);
}

TEST(TestBLDCMotor, CommutatedMotorRunsAtAboutTheDCSpeed)
{
    const BLDCMotorParameters m;
    BLDCMotor motor(m);
    BLDCRotor rotor;
    const real_t dt = 1.0e-6;
    for (int k = 0; k < 2000000; ++k)
    {
        const real_t T = motor.step(12.0, rotor.theta, rotor.w, dt);
        rotor.step(m, T, 0.0, dt);
    }

    // The torque ripples with the commutation, so the speed is within a few percent
    EXPECT_GT(rotor.w, 0.0);
    EXPECT_NEAR(rotor.w, no_load_speed(m, 12.0), 0.05 * no_load_speed(m, 12.0));
}

TEST(TestPIController, LimitsItsOutputAndDoesNotWindUp)
{
    PIController pi(1.0, 10.0, 2.0);
    EXPECT_THROW(PIController(1.0, 1.0, 0.0), std::invalid_argument);

    // A large error for a long time saturates the output, and the integral stays put
    for (int k = 0; k < 1000; ++k)
    {
        EXPECT_LE(pi.update(5.0, 0.01), 2.0);
    }
    EXPECT_DOUBLE_EQ(pi.integral(), 0.0);

    // So the output comes off the limit as soon as the error changes sign
    EXPECT_LT(pi.update(-1.0, 0.01), 0.0);
}

TEST(TestPIController, FeedforwardAddsToTheOutput)
{
    PIController pi(2.0, 0.0, 10.0);
    EXPECT_DOUBLE_EQ(pi.update(1.0, 0.01, 3.0), 5.0);
    EXPECT_DOUBLE_EQ(pi.update(1.0, 0.01, 20.0), 10.0);
}

#include "bitsim/bitsim_consts.h"
#include "bitsim/bitsim_types.h"
#include "bitsim/control/fixed_wing_autopilot.h"
#include "bitsim/control/fixed_wing_mission.h"
#include "bitsim/dynamics/fixed_wing_dynamics.h"
#include "bitsim/dynamics/fixed_wing_trim.h"

#include <gtest/gtest.h>

#include <cmath>
#include <map>
#include <string>
#include <vector>

namespace
{
using bitsim::Mat;
using bitsim::real_t;
using namespace bitsim::dynamics;
using namespace bitsim::control;

const real_t PI = bitsim::consts::maths::PI;
const real_t G = bitsim::consts::maths::G;

FixedWingParameters aerosonde() { return FixedWingParameters::aerosonde(); }

// On its wheels, still, at the start of a runway heading north
FixedWingDynamics on_the_ground(const FixedWingParameters &p)
{
    FixedWingState x = FixedWingState::Zero();
    x[fw::PD] = -p.gear.front().position[2];
    FixedWingDynamics aircraft(p, x);
    for (int k = 0; k < 200; ++k)
    {
        aircraft.step({}, 0.01);
    }
    return aircraft;
}
} // namespace

TEST(TestFixedWingDynamics, FallsFreelyWhenStillInTheAir)
{
    FixedWingState x = FixedWingState::Zero();
    x[fw::PD] = -100.0;
    const FixedWingDynamics aircraft(aerosonde(), x);
    const auto dx = aircraft.derivative(x, {});
    EXPECT_NEAR(dx[fw::U], 0.0, 1.0e-9);
    EXPECT_NEAR(dx[fw::W], G, 1.0e-9);
    EXPECT_NEAR(dx[fw::Q], 0.0, 1.0e-9);
}

TEST(TestFixedWingDynamics, AirDataComeFromTheVelocityThroughTheAir)
{
    FixedWingState x = FixedWingState::Zero();
    x[fw::PD] = -100.0;
    x[fw::U] = 20.0;
    x[fw::W] = 2.0;
    FixedWingDynamics aircraft(aerosonde(), x);
    EXPECT_NEAR(aircraft.airspeed(), std::hypot(20.0, 2.0), 1.0e-12);
    EXPECT_NEAR(aircraft.alpha(), std::atan2(2.0, 20.0), 1.0e-12);
    EXPECT_NEAR(aircraft.beta(), 0.0, 1.0e-12);

    // A wind from the east, blowing west, of 3 m/s: the air comes at the aircraft from its right
    aircraft.set_wind({0.0, -3.0, 0.0});
    EXPECT_NEAR(aircraft.airspeed(), std::sqrt(400.0 + 9.0 + 4.0), 1.0e-12);
    EXPECT_GT(aircraft.beta(), 0.0);
    EXPECT_NEAR(aircraft.ground_speed(), 20.0 * std::cos(0.0), 1.0e-12);
}

TEST(TestFixedWingTrim, LevelFlightWithinTheAircraftsLimits)
{
    const auto p = aerosonde();
    for (const real_t Va : {18.0, 22.0, 25.0, 28.0})
    {
        const auto t = trim_level_flight(p, Va);
        EXPECT_TRUE(t.converged) << Va;
        EXPECT_LT(t.residual, 1.0e-8) << Va;
        // The lift carries the weight
        const FixedWingDynamics aircraft(p, t.state);
        const auto f = aircraft.forces(t.state, t.controls);
        EXPECT_NEAR(f.airspeed, Va, 1.0e-9);
        EXPECT_GT(t.alpha, 0.0);
    }
    // Faster, the angle of attack and the up elevator are smaller and the throttle larger
    const auto slow = trim_level_flight(p, 18.0);
    const auto fast = trim_level_flight(p, 25.0);
    EXPECT_GT(slow.alpha, fast.alpha);
    EXPECT_LT(slow.controls.elevator, fast.controls.elevator);
    EXPECT_LT(slow.controls.throttle, fast.controls.throttle);

    // Too slow for the elevator, too fast for the motor
    EXPECT_FALSE(trim_level_flight(p, 14.0).converged);
    EXPECT_FALSE(trim_level_flight(p, 34.0).converged);
}

TEST(TestFixedWingTrim, TrimmedFlightStaysTrimmed)
{
    const auto p = aerosonde();
    const auto t = trim_level_flight(p, 25.0);
    FixedWingDynamics aircraft(p, t.state);
    for (int k = 0; k < 2000; ++k) // 20 s
    {
        aircraft.step(t.controls, 0.01);
    }
    EXPECT_NEAR(aircraft.altitude(), 100.0, 0.05);
    EXPECT_NEAR(aircraft.airspeed(), 25.0, 0.01);
    EXPECT_NEAR(aircraft.state()[fw::PN], 25.0 * 20.0, 0.5);
}

TEST(TestFixedWingTrim, TheLinearModelPredictsSmallMotions)
{
    // An elevator pulse of 0.02 rad for 0.5 s, then 1.5 s more: the nonlinear model and the
    // linear one end where each other does
    const auto p = aerosonde();
    const auto t = trim_level_flight(p, 25.0);
    const auto lin = linearise(p, t);
    FixedWingDynamics aircraft(p, t.state);
    Mat<real_t, 12, 1> dx = Mat<real_t, 12, 1>::Zero();
    const real_t dt = 0.001;
    for (int k = 0; k < 2000; ++k)
    {
        Mat<real_t, 4, 1> du = Mat<real_t, 4, 1>::Zero();
        FixedWingControls u = t.controls;
        if (k < 500)
        {
            du[0] = 0.02;
            u.elevator += 0.02;
        }
        aircraft.step(u, dt);
        dx += dt * (lin.A * dx + lin.B * du);
    }
    const FixedWingState change = aircraft.state() - t.state;
    for (const int i : {fw::U, fw::W, fw::Q, fw::THETA, fw::PD})
    {
        EXPECT_NEAR(change[i], dx[i], 0.05 * std::abs(dx[i]) + 1.0e-3) << "state " << i;
    }
    // The pulse pitches the nose down: positive elevator is trailing edge down
    EXPECT_LT(dx[fw::THETA], 0.0);
}

TEST(TestFixedWingTrim, TheModesOfASmallUAV)
{
    const auto p = aerosonde();
    const auto modes = flight_modes(linearise(p, trim_level_flight(p, 25.0)));
    std::map<std::string, FlightMode> by_name;
    for (const auto &m : modes)
    {
        by_name[m.name] = m;
    }
    for (const char *name : {"short period", "phugoid", "Dutch roll", "roll", "spiral"})
    {
        ASSERT_EQ(by_name.count(name), 1u) << name;
    }
    const auto &sp = by_name["short period"];
    const auto &ph = by_name["phugoid"];
    // The short period is fast and well damped, the phugoid slow and lightly damped
    EXPECT_GT(sp.natural_frequency, 10.0 * ph.natural_frequency);
    EXPECT_GT(sp.damping, 0.3);
    EXPECT_GT(ph.damping, 0.0);
    EXPECT_LT(ph.damping, sp.damping);
    // The roll mode is fast and stable, the Dutch roll an oscillation that decays
    EXPECT_LT(by_name["roll"].eigenvalue.real(), -5.0);
    EXPECT_GT(by_name["Dutch roll"].damping, 0.1);
    // The spiral is slow, whichever way it goes
    EXPECT_LT(std::abs(by_name["spiral"].eigenvalue.real()), 0.2);
}

TEST(TestFixedWingDynamics, StandsAndBrakesOnItsWheels)
{
    const auto p = aerosonde();
    auto aircraft = on_the_ground(p);
    EXPECT_TRUE(aircraft.on_ground());
    const auto f = aircraft.forces(aircraft.state(), {});
    EXPECT_NEAR(f.ground_load, p.mass * G, 0.01 * p.mass * G);
    EXPECT_NEAR(aircraft.ground_speed(), 0.0, 1.0e-3);

    // A little throttle against the brakes: it stays
    FixedWingControls hold;
    hold.throttle = 0.3;
    hold.brake = 1.0;
    const real_t start = aircraft.state()[fw::PN];
    for (int k = 0; k < 300; ++k)
    {
        aircraft.step(hold, 0.01);
    }
    EXPECT_NEAR(aircraft.state()[fw::PN], start, 0.05);

    // Without them it rolls
    hold.brake = 0.0;
    for (int k = 0; k < 300; ++k)
    {
        aircraft.step(hold, 0.01);
    }
    EXPECT_GT(aircraft.state()[fw::PN] - start, 1.0);
}

TEST(TestFixedWingAutopilot, HoldsCourseAltitudeAndAirspeed)
{
    const auto p = aerosonde();
    FixedWingAutopilot autopilot(p);
    FixedWingDynamics aircraft(p, autopilot.trim().state);
    // A quarter turn, 20 m up and 5 m/s slower, all at once
    const FixedWingCommands commands{20.0, 120.0, PI / 2.0};
    real_t max_beta = 0.0;
    real_t max_turning_beta = 0.0;
    real_t banked_for = 0.0;
    for (int k = 0; k < 4000; ++k) // 40 s
    {
        aircraft.step(autopilot.update(aircraft, commands, 0.01), 0.01);
        max_beta = std::max(max_beta, std::abs(aircraft.beta()));
        // In the steady part of the turn: banked for more than 1.5 s, and not rolling out
        banked_for = std::abs(aircraft.state()[fw::PHI]) > 0.4 ? banked_for + 0.01 : 0.0;
        if (banked_for > 1.5 && std::abs(aircraft.state()[fw::P]) < 0.1)
        {
            max_turning_beta = std::max(max_turning_beta, std::abs(aircraft.beta()));
        }
    }
    EXPECT_NEAR(std::remainder(aircraft.course() - PI / 2.0, 2.0 * PI), 0.0, 0.01);
    EXPECT_NEAR(aircraft.altitude(), 120.0, 0.5);
    EXPECT_NEAR(aircraft.airspeed(), 20.0, 0.2);
    EXPECT_NEAR(aircraft.state()[fw::PHI], 0.0, 0.01);
    // The turn is coordinated: the rudder holds the sideslip within a degree. Rolling in and out at
    // full aileron, adverse yaw slips it a few degrees for a moment
    EXPECT_LT(max_turning_beta, 1.0 * PI / 180.0);
    EXPECT_LT(max_beta, 6.0 * PI / 180.0);
}

TEST(TestFixedWingMission, TakesOffFliesTheRouteAndLands)
{
    const auto p = aerosonde();
    for (const real_t crosswind : {0.0, -4.0})
    {
        FixedWingAutopilot autopilot(p);
        const Runway runway{0.0, 0.0, 0.0, 400.0, 20.0};
        FixedWingMission mission(autopilot, runway,
                                 {{600.0, 0.0, 50.0}, {600.0, 400.0, 60.0}, {-650.0, 300.0, 45.0}});
        auto aircraft = on_the_ground(p);
        aircraft.set_wind({0.0, crosswind, 0.0});

        std::vector<MissionPhase> phases{mission.phase()};
        while (aircraft.time() < 400.0 && mission.phase() != MissionPhase::STOPPED)
        {
            aircraft.step(mission.update(aircraft, 0.01), 0.01);
            if (mission.phase() != phases.back())
            {
                phases.push_back(mission.phase());
            }
        }
        // Every phase, in order
        EXPECT_EQ(phases, (std::vector<MissionPhase>{MissionPhase::TAKEOFF_ROLL,
                                                     MissionPhase::ROTATE, MissionPhase::ENROUTE,
                                                     MissionPhase::APPROACH, MissionPhase::FLARE,
                                                     MissionPhase::ROLLOUT, MissionPhase::STOPPED}))
            << "crosswind " << crosswind;
        ASSERT_TRUE(mission.touchdown().has_value());
        const auto &td = *mission.touchdown();
        EXPECT_NEAR(td.along, 60.0, 20.0);
        EXPECT_LT(std::abs(td.across), 1.0);
        EXPECT_LT(td.sink_rate, 1.5);
        const auto end =
            mission.runway_coordinates(aircraft.state()[fw::PN], aircraft.state()[fw::PE]);
        EXPECT_GT(end[0], td.along);
        EXPECT_LT(end[0], runway.length);
        EXPECT_LT(std::abs(end[1]), 1.0);
    }
}

// A fixed wing aircraft turns through 90 degrees and climbs 20 m, flown by an LQR, by model
// predictive control and by the autopilot of fixed_wing_flight.
//
// Usage: fixed_wing_lqr_mpc [--record DIR]
//
// The LQR and MPC are designed on the same linear model, at the trim of level flight, with the
// same weights. MPC also keeps the bank within 25 degrees and the pitch within 15 degrees of the
// trim, and the control surfaces within their limits. The autopilot is the cascade of loops of
// fixed_wing_flight, with its own limits on bank and pitch. --record DIR writes a picture of the
// three every 0.1 s of simulated time to DIR, which needs bitsim built with OpenCV.
#include "bitsim/bitsim_config.h"
#include "bitsim/bitsim_types.h"
#include "bitsim/control/fixed_wing_autopilot.h"
#include "bitsim/control/linear_mpc.h"
#include "bitsim/control/lqr.h"
#include "bitsim/dynamics/fixed_wing_dynamics.h"
#include "bitsim/dynamics/fixed_wing_trim.h"

#ifdef BITSIM_OPENCV
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#endif

#include <array>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace fixed_wing_lqr_mpc
{

using bitsim::index_t;
using bitsim::real_t;
using bitsim::uint_t;
using bitsim::control::Matrix;
using bitsim::control::Vector;
using bitsim::dynamics::FixedWingControls;
using bitsim::dynamics::FixedWingDynamics;
using bitsim::dynamics::FixedWingParameters;
namespace fw = bitsim::dynamics::fw;

const real_t PI = 3.14159265358979323846;
const real_t DEG = PI / 180.0;
const real_t STEP = 0.01;           // of the dynamics [s]
const real_t CONTROL_PERIOD = 0.05; // all three run at 20 Hz [s]
const real_t AIRSPEED = 25.0;       // [m/s]
const real_t ALTITUDE = 100.0;      // [m]
const real_t TURN_AT = 5.0;         // the heading goes from 0 to 90 degrees [s]
const real_t CLIMB_AT = 30.0;       // the altitude goes from 100 m to 120 m [s]
const real_t DURATION = 60.0;       // [s]
const uint_t HORIZON = 60;          // MPC plans 3 s
const real_t MAX_BANK = 25.0 * DEG;
const real_t MAX_PITCH = 15.0 * DEG; // from the trim
const real_t PENALTY = 2.0e4;        // of a bank or a pitch beyond its bound
const real_t RECORD_RATE = 10.0;     // [pictures/s]

// The states of the LQR and MPC: the twelve of the aircraft but its position north and east,
// which nothing depends on
const std::array<int, 10> STATES{fw::PD,    fw::U,   fw::V, fw::W, fw::PHI,
                                 fw::THETA, fw::PSI, fw::P, fw::Q, fw::R};

// What happened to one aircraft
struct Track
{
    std::string name;
    std::vector<std::array<real_t, 2>> ground; // north, east [m]
    std::vector<real_t> bank;                  // [rad]
    std::vector<real_t> pitch;                 // [rad]
    std::vector<real_t> heading;               // [rad]
    std::vector<real_t> altitude;              // [m]
};

Vector controls_vector(const FixedWingControls &c)
{
    Vector u(4);
    u << c.elevator, c.aileron, c.rudder, c.throttle;
    return u;
}

FixedWingControls to_controls(const Vector &u)
{
    FixedWingControls c;
    c.elevator = u[0];
    c.aileron = u[1];
    c.rudder = u[2];
    c.throttle = u[3];
    return c;
}

// The deviation of the aircraft from the trim at the reference, the heading within pi of it
Vector deviation(const FixedWingDynamics &aircraft, const bitsim::dynamics::FixedWingTrim &trim,
                 real_t heading_ref, real_t altitude_ref)
{
    Vector dx(10);
    for (uint_t i = 0; i < STATES.size(); ++i)
    {
        dx[static_cast<index_t>(i)] = aircraft.state()[STATES[i]] - trim.state[STATES[i]];
    }
    dx[0] = aircraft.state()[fw::PD] + altitude_ref;
    dx[6] = std::remainder(aircraft.state()[fw::PSI] - heading_ref, 2.0 * PI);
    return dx;
}

void record_track(Track &track, const FixedWingDynamics &a)
{
    track.ground.push_back({a.state()[fw::PN], a.state()[fw::PE]});
    track.bank.push_back(a.state()[fw::PHI]);
    track.pitch.push_back(a.state()[fw::THETA]);
    track.heading.push_back(a.state()[fw::PSI]);
    track.altitude.push_back(a.altitude());
}

#ifdef BITSIM_OPENCV
const std::array<cv::Scalar, 3> COLOURS{cv::Scalar(60, 60, 220), cv::Scalar(200, 120, 30),
                                        cv::Scalar(40, 150, 40)};

// The three at sample k: the ground tracks seen from above, and below them the bank and the
// altitude over time
cv::Mat draw(const std::vector<Track> &tracks, uint_t k, real_t sample_period)
{
    const int W = 960;
    const int H = 540;
    cv::Mat picture(H, W, CV_8UC3, cv::Scalar(250, 250, 250));
    const auto text =
        [&](const std::string &s, cv::Point at, const cv::Scalar &colour, double size = 0.5)
    { cv::putText(picture, s, at, cv::FONT_HERSHEY_SIMPLEX, size, colour, 1, cv::LINE_AA); };

    // the tracks from above: east to the right, north up, at one scale for both, from -50 m to
    // 1500 m east and from 0 to 260 m north
    const cv::Rect map(20, 40, 920, 160);
    const real_t scale = map.width / 1550.0; // pixels per metre
    cv::rectangle(picture, map, cv::Scalar(200, 200, 200), 1);
    const auto ground_px = [&](real_t north, real_t east)
    {
        return cv::Point(map.x + static_cast<int>((east + 50.0) * scale),
                         map.y + map.height - 8 - static_cast<int>(north * scale));
    };
    text("from above", {map.x, map.y - 12}, cv::Scalar(60, 60, 60));
    for (uint_t i = 0; i < tracks.size(); ++i)
    {
        const auto &g = tracks[i].ground;
        for (uint_t j = 1; j <= k && j < g.size(); ++j)
        {
            cv::line(picture, ground_px(g[j - 1][0], g[j - 1][1]), ground_px(g[j][0], g[j][1]),
                     COLOURS[i], 2, cv::LINE_AA);
        }
        const auto here = ground_px(g[k][0], g[k][1]);
        const real_t psi = tracks[i].heading[k];
        const cv::Point nose(here.x + static_cast<int>(12 * std::sin(psi)),
                             here.y - static_cast<int>(12 * std::cos(psi)));
        cv::circle(picture, here, 4, COLOURS[i], -1, cv::LINE_AA);
        cv::line(picture, here, nose, COLOURS[i], 2, cv::LINE_AA);
    }

    // the bank and the altitude over time
    const auto plot = [&](const cv::Rect &box, const std::string &title, real_t lo, real_t hi,
                          const std::vector<real_t> &marks, auto value)
    {
        cv::rectangle(picture, box, cv::Scalar(200, 200, 200), 1);
        text(title, {box.x, box.y - 8}, cv::Scalar(60, 60, 60));
        const auto px = [&](uint_t j, real_t v)
        {
            return cv::Point(box.x + static_cast<int>(j * sample_period / DURATION * box.width),
                             box.y + box.height -
                                 static_cast<int>((v - lo) / (hi - lo) * box.height));
        };
        for (real_t m : marks)
        {
            for (int x = box.x; x < box.x + box.width; x += 10)
            {
                const int y = px(0, m).y;
                cv::line(picture, {x, y}, {x + 5, y}, cv::Scalar(150, 150, 150), 1);
            }
        }
        for (uint_t i = 0; i < tracks.size(); ++i)
        {
            for (uint_t j = 1; j <= k; ++j)
            {
                cv::line(picture, px(j - 1, value(tracks[i], j - 1)), px(j, value(tracks[i], j)),
                         COLOURS[i], 2, cv::LINE_AA);
            }
        }
    };
    plot(cv::Rect(20, 250, 440, 250), "bank [deg], with MPC's bound", -30.0, 80.0, {-25.0, 25.0},
         [](const Track &t, uint_t j) { return t.bank[j] / DEG; });
    plot(cv::Rect(500, 250, 440, 250), "altitude [m]", 90.0, 125.0, {100.0, 120.0},
         [](const Track &t, uint_t j) { return t.altitude[j]; });

    char status[120];
    std::snprintf(status, sizeof(status), "t %4.1f s", k * sample_period);
    text(status, {20, 530}, cv::Scalar(30, 30, 30));
    for (uint_t i = 0; i < tracks.size(); ++i)
    {
        text(tracks[i].name, {140 + 110 * static_cast<int>(i), 530}, COLOURS[i]);
    }
    return picture;
}

void record(const std::vector<Track> &tracks, const std::string &directory, real_t sample_period)
{
    std::filesystem::create_directories(directory);
    const auto every = static_cast<uint_t>(std::lround(1.0 / (RECORD_RATE * sample_period)));
    uint_t n = 0;
    for (uint_t k = 0; k < tracks.front().ground.size(); k += every)
    {
        char name[32];
        std::snprintf(name, sizeof(name), "/frame_%04u.png", static_cast<unsigned>(n++));
        cv::imwrite(directory + name, draw(tracks, k, sample_period));
    }
    std::cout << "Wrote " << n << " pictures to " << directory << std::endl;
}
#endif

} // namespace fixed_wing_lqr_mpc

int main(int argc, char **argv)
{
    using namespace fixed_wing_lqr_mpc;

    std::string record_dir;
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "--record" && i + 1 < argc)
        {
            record_dir = argv[++i];
        }
        else
        {
            std::cerr << "Usage: fixed_wing_lqr_mpc [--record DIR]\n";
            return 1;
        }
    }

    // the trim of level flight, and the linear model about it without north and east
    const auto p = FixedWingParameters::aerosonde();
    const auto trim = bitsim::dynamics::trim_level_flight(p, AIRSPEED, ALTITUDE, 0.0);
    const auto linear = bitsim::dynamics::linearise(p, trim);
    Matrix A(10, 10);
    Matrix B(10, 4);
    for (uint_t i = 0; i < STATES.size(); ++i)
    {
        for (uint_t j = 0; j < STATES.size(); ++j)
        {
            A(i, j) = linear.A(STATES[i], STATES[j]);
        }
        for (uint_t j = 0; j < 4; ++j)
        {
            B(i, j) = linear.B(STATES[i], j);
        }
    }
    const auto model = bitsim::control::discretise(A, B, CONTROL_PERIOD);

    // Bryson's rule: 0.25 m of altitude, 1 m/s, 0.2 rad of bank, 0.1 rad of pitch and of heading,
    // 0.5 rad/s; 0.1 rad of each control surface and 0.2 of throttle
    Vector q(10);
    q << 16, 1, 1, 1, 25, 100, 100, 4, 4, 4;
    Vector r(4);
    r << 100, 100, 100, 25;
    const Matrix Q = q.asDiagonal().toDenseMatrix();
    const Matrix R = r.asDiagonal().toDenseMatrix();
    const auto lqr = bitsim::control::dlqr(model.A, model.B, Q, R);
    const Vector u_trim = controls_vector(trim.controls);

    // MPC: the same, with the limits of the controls and the bounds on the bank and the pitch,
    // all as deviations from the trim
    const real_t inf = std::numeric_limits<real_t>::infinity();
    bitsim::control::LinearMPC mpc(10, 4, HORIZON);
    mpc.set_model(model.A, model.B);
    mpc.set_weights(Q, R);
    Vector u_lo(4);
    u_lo << -p.max_elevator, -p.max_aileron, -p.max_rudder, 0.0;
    Vector u_hi(4);
    u_hi << p.max_elevator, p.max_aileron, p.max_rudder, p.max_throttle;
    mpc.set_input_bounds(u_lo - u_trim, u_hi - u_trim);
    Vector x_lo = Vector::Constant(10, -inf);
    Vector x_hi = Vector::Constant(10, inf);
    x_lo[4] = -MAX_BANK;
    x_hi[4] = MAX_BANK;
    x_lo[5] = -MAX_PITCH;
    x_hi[5] = MAX_PITCH;
    mpc.set_state_bounds(x_lo, x_hi, PENALTY);

    bitsim::control::FixedWingAutopilot autopilot(p);

    std::array<FixedWingDynamics, 3> aircraft{FixedWingDynamics(p, trim.state),
                                              FixedWingDynamics(p, trim.state),
                                              FixedWingDynamics(p, trim.state)};
    std::vector<Track> tracks{{"LQR", {}, {}, {}, {}, {}},
                              {"MPC", {}, {}, {}, {}, {}},
                              {"autopilot", {}, {}, {}, {}, {}}};
    std::array<FixedWingControls, 3> controls{trim.controls, trim.controls, trim.controls};
    uint_t unsolved = 0;
    uint_t n_solves = 0;

    const auto steps = static_cast<uint_t>(std::lround(DURATION / STEP));
    const auto every = static_cast<uint_t>(std::lround(CONTROL_PERIOD / STEP));
    for (uint_t k = 0; k <= steps; ++k)
    {
        const real_t t = k * STEP;
        const real_t heading_ref = t < TURN_AT ? 0.0 : 90.0 * DEG;
        const real_t altitude_ref = t < CLIMB_AT ? ALTITUDE : ALTITUDE + 20.0;
        if (k % every == 0)
        {
            const Vector lqr_dx = deviation(aircraft[0], trim, heading_ref, altitude_ref);
            controls[0] = to_controls(u_trim - lqr.K * lqr_dx);

            const Vector mpc_dx = deviation(aircraft[1], trim, heading_ref, altitude_ref);
            controls[1] = to_controls(u_trim + mpc.solve(mpc_dx));
            unsolved += mpc.status() == bitsim::control::LinearMPC::QPStatus::SOLVED ? 0 : 1;
            ++n_solves;

            bitsim::control::FixedWingCommands commands;
            commands.airspeed = AIRSPEED;
            commands.altitude = altitude_ref;
            commands.course = heading_ref;
            controls[2] = autopilot.update(aircraft[2], commands, CONTROL_PERIOD);

            for (uint_t i = 0; i < 3; ++i)
            {
                record_track(tracks[i], aircraft[i]);
            }
        }
        for (uint_t i = 0; i < 3; ++i)
        {
            aircraft[i].step(controls[i], STEP);
        }
    }

    // the turn, from TURN_AT to CLIMB_AT, and the climb, from CLIMB_AT on
    std::printf("%-10s  %9s  %10s  %22s  %18s  %14s  %10s  %20s\n", "", "max bank", "max pitch",
                "turn: heading overshoot", "within 2 deg after", "lowest", "climb: max",
                "within 0.5 m after");
    for (const auto &track : tracks)
    {
        real_t bank = 0.0;
        real_t pitch = 0.0;
        real_t overshoot = 0.0;
        real_t turned = -1.0;
        real_t lowest = ALTITUDE;
        real_t top = 0.0;
        real_t climbed = -1.0;
        for (uint_t j = 0; j < track.bank.size(); ++j)
        {
            const real_t t = j * CONTROL_PERIOD;
            bank = std::max(bank, std::abs(track.bank[j]));
            pitch = std::max(pitch, std::abs(track.pitch[j]));
            if (t > TURN_AT && t < CLIMB_AT)
            {
                const real_t off = std::remainder(track.heading[j] - 90.0 * DEG, 2.0 * PI);
                overshoot = std::max(overshoot, off);
                lowest = std::min(lowest, track.altitude[j]);
                if (turned < 0.0 && std::abs(off) < 2.0 * DEG)
                {
                    turned = t - TURN_AT;
                }
            }
            if (t > CLIMB_AT)
            {
                top = std::max(top, track.altitude[j]);
                if (climbed < 0.0 && std::abs(track.altitude[j] - ALTITUDE - 20.0) < 0.5)
                {
                    climbed = t - CLIMB_AT;
                }
            }
        }
        std::printf(
            "%-10s  %5.1f deg  %6.1f deg  %18.1f deg  %16.1f s  %12.1f m  %8.1f m  %18.1f s\n",
            track.name.c_str(), bank / DEG, pitch / DEG, overshoot / DEG, turned, lowest, top,
            climbed);
    }
    std::printf("\nMPC solved %u QPs, %u of them not to the tolerance\n",
                static_cast<unsigned>(n_solves), static_cast<unsigned>(unsolved));

    if (!record_dir.empty())
    {
#ifdef BITSIM_OPENCV
        record(tracks, record_dir, CONTROL_PERIOD);
#else
        std::cerr << "--record needs bitsim built with OpenCV\n";
        return 1;
#endif
    }

    // what the example shows: MPC keeps to its bound on the bank, and gets there
    real_t mpc_bank = 0.0;
    for (const real_t b : tracks[1].bank)
    {
        mpc_bank = std::max(mpc_bank, std::abs(b));
    }
    const real_t final_heading = tracks[1].heading.back();
    const bool ok = mpc_bank < MAX_BANK + 1.0 * DEG &&
                    std::abs(std::remainder(final_heading - 90.0 * DEG, 2.0 * PI)) < 1.0 * DEG &&
                    std::abs(tracks[1].altitude.back() - ALTITUDE - 20.0) < 0.5;
    return ok ? 0 : 1;
}

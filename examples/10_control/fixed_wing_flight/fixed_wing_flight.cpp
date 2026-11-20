// A fixed wing UAV, the Aerosonde of Beard and McLain, takes off from a runway, flies a circuit of
// waypoints with an autopilot, and lands back on the runway. First the example finds the
// aircraft's trim for level flight at a range of airspeeds and the modes of its motion about one
// of them; the autopilot's gains come from that linear model.
//
// Usage: fixed_wing_flight [--wind N E] [--render] [--record DIR]
//
// --wind N E blows a steady wind towards the north and the east [m/s]. --render shows the flight
// in a window, in real time. --record DIR also writes the window to DIR, ten pictures a second,
// during the takeoff and the landing, below 25 m.

#include "bitrl/bitrl_config.h"
#include "bitrl/bitrl_consts.h"
#include "bitrl/bitrl_types.h"
#include "bitrl/control/fixed_wing_autopilot.h"
#include "bitrl/control/fixed_wing_mission.h"
#include "bitrl/dynamics/fixed_wing_dynamics.h"
#include "bitrl/dynamics/fixed_wing_trim.h"
#include "bitrl/utils/io/print_utils.h"

#if defined(BITRL_CHRONO) && defined(BITRL_IRRLICHT)
#include "bitrl/utils/render/irrlicht_scene.h"
#include "bitrl/utils/render/irrlicht_utils.h"
#include <chrono/assets/ChVisualShapeBox.h>
#include <chrono/assets/ChVisualShapeCylinder.h>
#include <chrono/physics/ChBody.h>
#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChSystemNSC.h>
#include <chrono_irrlicht/ChVisualSystemIrrlicht.h>
#define FIXED_WING_RENDER
#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace fixed_wing_flight
{

using bitrl::real_t;
using bitrl::uint_t;
using namespace bitrl::dynamics;
using namespace bitrl::control;
using bitrl::utils::io::print_title;

const real_t PI = bitrl::consts::maths::PI;
const real_t DEG = PI / 180.0;

const real_t STEP = 0.01;            // [s]
const real_t PRINT_PERIOD = 10.0;    // [s]
const real_t TIME_LIMIT = 400.0;     // [s]
const real_t RENDER_RATE = 30.0;     // [frames/s]
const real_t RECORD_RATE = 10.0;     // [pictures/s]
const real_t DESIGN_AIRSPEED = 25.0; // [m/s]

// A runway 400 m long running north, and a circuit around it: north, east, back south past the
// runway, and onto the final approach from the south west
const Runway RUNWAY{0.0, 0.0, 0.0, 400.0, 20.0};
const std::vector<MissionWaypoint> ROUTE{
    {700.0, 0.0, 60.0}, {700.0, 500.0, 80.0}, {-450.0, 500.0, 80.0}, {-700.0, 250.0, 50.0}};

struct Options
{
    real_t wind_north{0.0};
    real_t wind_east{0.0};
    bool render{false};
    std::string record;
};

// ---------------------------------------------------------------------------
// Trim and modes
// ---------------------------------------------------------------------------

void show_trims(const FixedWingParameters &p)
{
    print_title("1. Trim for straight and level flight");
    std::cout << "  Va [m/s]  alpha [deg]  elevator [deg]  throttle  trim\n";
    for (real_t Va = 14.0; Va <= 34.0 + 1e-9; Va += 4.0)
    {
        const auto t = trim_level_flight(p, Va);
        std::string why = "yes";
        if (!t.converged)
        {
            why = t.controls.throttle > 1.0 ? "no: needs more than full throttle"
                  : std::abs(t.controls.elevator) > p.max_elevator
                      ? "no: needs more elevator than it has"
                      : "no";
        }
        std::cout << std::fixed << std::setprecision(1) << std::setw(10) << Va << std::setw(13)
                  << t.alpha / DEG << std::setw(16) << t.controls.elevator / DEG
                  << std::setprecision(2) << std::setw(10) << t.controls.throttle << "  " << why
                  << "\n";
    }
}

void show_modes(const FixedWingParameters &p)
{
    print_title("2. The modes of its motion at 25 m/s");
    const auto trim = trim_level_flight(p, DESIGN_AIRSPEED);
    const auto modes = flight_modes(linearise(p, trim));
    std::cout << "  mode            eigenvalue          wn [rad/s]  damping  period [s]  "
                 "time constant [s]\n";
    for (const auto &m : modes)
    {
        std::ostringstream eig;
        eig << std::fixed << std::setprecision(3) << m.eigenvalue.real();
        if (std::abs(m.eigenvalue.imag()) > 1e-9)
        {
            eig << " +/- " << std::abs(m.eigenvalue.imag()) << "i";
        }
        std::ostringstream period;
        if (m.period > 0.0)
        {
            period << std::fixed << std::setprecision(2) << m.period;
        }
        else
        {
            period << "-";
        }
        std::cout << "  " << std::left << std::setw(14) << m.name << std::setw(20) << eig.str()
                  << std::right << std::fixed << std::setprecision(3) << std::setw(10)
                  << m.natural_frequency << std::setw(9) << m.damping << std::setw(12)
                  << period.str() << std::setprecision(2) << std::setw(15) << m.time_constant
                  << (m.time_constant < 0 ? "  it grows" : "") << "\n";
    }
}

void show_gains(const FixedWingAutopilot &autopilot)
{
    print_title("3. Autopilot gains, from the linear model at 25 m/s");
    const auto &g = autopilot.gains();
    std::cout << std::fixed << std::setprecision(3) << "  roll:      kp " << g.roll_kp << ", kd "
              << g.roll_kd << "\n"
              << "  course:    kp " << g.course_kp << ", ki " << g.course_ki << "\n"
              << "  sideslip:  kp " << g.sideslip_kp << ", ki " << g.sideslip_ki << "\n"
              << "  pitch:     kp " << g.pitch_kp << ", kd " << g.pitch_kd << ", DC gain "
              << g.pitch_dc_gain << "\n"
              << "  altitude:  kp " << g.altitude_kp << ", ki " << g.altitude_ki << "\n"
              << "  airspeed:  kp " << g.throttle_kp << ", ki " << g.throttle_ki
              << " with the throttle; kp " << g.airspeed_pitch_kp << ", ki " << g.airspeed_pitch_ki
              << " with the pitch\n";
}

// ---------------------------------------------------------------------------
// The window
// ---------------------------------------------------------------------------

#ifdef FIXED_WING_RENDER
using chrono::ChColor;
using chrono::ChVector3d;

// NED to Chrono's z up, a half turn about x: (n, e, d) -> (n, -e, -d)
const chrono::ChMatrix33d NED_TO_Z_UP(ChVector3d(1, 0, 0), ChVector3d(0, -1, 0),
                                      ChVector3d(0, 0, -1));
ChVector3d to_z_up(real_t n, real_t e, real_t d) { return {n, -e, -d}; }

class Display
{
  public:
    Display(const FixedWingMission &mission, const FixedWingParameters &p,
            const std::string &record)
        : recorder_(record, RECORD_RATE)
    {
        system_.SetGravitationalAcceleration(ChVector3d(0, 0, 0));
        add_box_({0, 0, -0.05}, {4000, 4000, 0.1}, ChColor(0.42f, 0.55f, 0.36f));
        // The runway, its centre line and the threshold's bars
        const real_t L = RUNWAY.length;
        add_box_({L / 2, 0, 0.005}, {L, RUNWAY.width, 0.01}, ChColor(0.28f, 0.28f, 0.30f));
        for (real_t x = 20.0; x < L - 10.0; x += 25.0)
        {
            add_box_({x, 0, 0.012}, {12, 0.4, 0.01}, ChColor(0.92f, 0.92f, 0.92f));
        }
        for (real_t y = -7.5; y <= 7.5; y += 2.5)
        {
            add_box_({5, y, 0.012}, {8, 1.0, 0.01}, ChColor(0.92f, 0.92f, 0.92f));
        }
        // A pole under each point of the route, as high as it
        for (std::size_t i = 1; i + 1 < mission.points().size(); ++i)
        {
            const auto &w = mission.points()[i];
            add_box_(to_z_up(w.north, w.east, -0.5 * w.altitude), {1.5, 1.5, w.altitude},
                     ChColor(0.9f, 0.55f, 0.15f));
        }
        aircraft_ = make_aircraft_(p);
        system_.Add(aircraft_);

        vis_.AttachSystem(&system_);
        vis_.SetWindowSize(1280, 720);
        vis_.SetWindowTitle("A fixed wing UAV: takeoff, circuit and landing");
        vis_.SetCameraVertical(chrono::CameraVerticalDir::Z);
        vis_.Initialize();
        vis_.AddCamera({-12, 4, 4}, {0, 0, 0});
        vis_.GetActiveCamera()->setFarValue(5000.0f);
        // A soft light: the typical lights wash the scene out
        vis_.AddLightDirectional(60.0, 60.0, ChColor(0.5f, 0.5f, 0.5f),
                                 ChColor(0.05f, 0.05f, 0.05f), ChColor(0.55f, 0.55f, 0.55f));
    }

    // Returns false once the window is closed
    bool show(const FixedWingDynamics &aircraft, const FixedWingMission &mission,
              const std::string &status, bool record)
    {
        if (!vis_.Run())
        {
            return false;
        }
        const auto &x = aircraft.state();
        const ChVector3d position = to_z_up(x[fw::PN], x[fw::PE], x[fw::PD]);
        const Eigen::Matrix3d R = FixedWingDynamics::rotation(x[fw::PHI], x[fw::THETA], x[fw::PSI]);
        chrono::ChMatrix33d Rc;
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                Rc(i, j) = R(i, j);
            }
        }
        aircraft_->SetPos(position);
        aircraft_->SetRot(chrono::ChMatrix33d(NED_TO_Z_UP * Rc * NED_TO_Z_UP));
        trail_.add(position);

        // The camera follows behind, to the right and above, along the course. In the drawing's
        // axes north is x and east is -y
        const real_t chi = aircraft.course();
        const ChVector3d ahead(std::cos(chi), -std::sin(chi), 0.0);
        const ChVector3d right(-std::sin(chi), -std::cos(chi), 0.0);
        vis_.UpdateCamera(position - ahead * 7.0 + right * 4.0 + ChVector3d(0, 0, 2.0), position);

        vis_.BeginScene();
        vis_.Render();
        // The route, and the line the aircraft follows now
        const auto &pts = mission.points();
        for (std::size_t i = 1; i < pts.size(); ++i)
        {
            const bool now = i == mission.leg() + 1;
            bitrl::utils::render::irr_draw_line(
                vis_, to_z_up(pts[i - 1].north, pts[i - 1].east, -pts[i - 1].altitude),
                to_z_up(pts[i].north, pts[i].east, -pts[i].altitude),
                now ? ChColor(0.2f, 0.9f, 1.0f) : ChColor(0.6f, 0.75f, 0.8f), now ? 2.0 : 1.0);
        }
        trail_.draw(vis_, ChColor(1.0f, 0.85f, 0.2f), 2.0);
        bitrl::utils::render::irr_draw_line(vis_, position,
                                            ChVector3d(position.x(), position.y(), 0.0),
                                            ChColor(0.2f, 0.2f, 0.2f), 1.0);
        bitrl::utils::render::draw_status_bar(vis_, status);
        vis_.EndScene();
        if (record)
        {
            recorder_.record(vis_, aircraft.time());
        }
        return true;
    }

  private:
    void add_box_(const ChVector3d &at, const ChVector3d &size, const ChColor &colour)
    {
        auto box = chrono_types::make_shared<chrono::ChBodyEasyBox>(size.x(), size.y(), size.z(),
                                                                    100.0, true, false);
        box->SetPos(at);
        box->SetFixed(true);
        box->GetVisualShape(0)->SetColor(colour);
        system_.Add(box);
    }

    // A fuselage, a wing, a tail and the wheels, in the drawing's axes: x ahead, y left, z up
    std::shared_ptr<chrono::ChBody> make_aircraft_(const FixedWingParameters &p)
    {
        auto body = chrono_types::make_shared<chrono::ChBody>();
        body->SetFixed(true);
        const auto add = [&](const ChVector3d &size, const ChVector3d &at, const ChColor &colour)
        {
            auto shape =
                chrono_types::make_shared<chrono::ChVisualShapeBox>(size.x(), size.y(), size.z());
            shape->SetColor(colour);
            body->AddVisualShape(shape, chrono::ChFrame<>(at));
        };
        const ChColor white(0.92f, 0.92f, 0.95f);
        const ChColor red(0.85f, 0.15f, 0.12f);
        add({1.7, 0.18, 0.18}, {0.05, 0, 0}, white);         // fuselage
        add({0.30, p.b, 0.03}, {0.0, 0, 0.06}, white);       // wing
        add({0.22, 0.95, 0.02}, {-0.85, 0, 0.04}, white);    // horizontal tail
        add({0.26, 0.02, 0.34}, {-0.88, 0, 0.21}, red);      // fin
        add({0.04, 0.05, 0.05}, {0.05, p.b / 2, 0.06}, red); // left wing tip
        add({0.04, 0.05, 0.05}, {0.05, -p.b / 2, 0.06}, ChColor(0.1f, 0.7f, 0.2f)); // right
        add({0.02, 0.55, 0.05}, {0.92, 0, 0}, ChColor(0.2f, 0.2f, 0.2f));           // propeller
        // The wheels, whose bottoms are where they touch the ground
        for (const auto &w : p.gear)
        {
            add({0.1, 0.04, 0.1}, {w.position[0], -w.position[1], -w.position[2] + 0.05},
                ChColor(0.1f, 0.1f, 0.1f));
        }
        return body;
    }

    chrono::ChSystemNSC system_;
    std::shared_ptr<chrono::ChBody> aircraft_;
    chrono::irrlicht::ChVisualSystemIrrlicht vis_;
    bitrl::utils::render::FrameRecorder recorder_;
    bitrl::utils::render::Trail trail_{1.0, 0.0};
};
#endif

// ---------------------------------------------------------------------------
// The flight
// ---------------------------------------------------------------------------

int fly(const FixedWingParameters &p, FixedWingAutopilot &autopilot, const Options &options)
{
    print_title("4. Takeoff, circuit and landing");
    FixedWingMission mission(autopilot, RUNWAY, ROUTE);
    std::cout << std::fixed << std::setprecision(0) << "  Runway " << RUNWAY.length
              << " m long, heading " << RUNWAY.heading / DEG
              << " deg. The route, north, east and altitude [m]:\n";
    for (std::size_t i = 1; i < mission.points().size(); ++i)
    {
        const auto &w = mission.points()[i];
        const char *what = i + 2 == mission.points().size()   ? "  final approach fix"
                           : i + 1 == mission.points().size() ? "  aim point"
                                                              : "";
        std::cout << std::fixed << std::setprecision(0) << "    " << std::setw(6) << w.north
                  << std::setw(7) << w.east << std::setw(5) << w.altitude << what << "\n";
    }
    std::cout << std::setprecision(1) << "  Wind towards the north " << options.wind_north
              << " m/s and the east " << options.wind_east << " m/s\n\n";

    // On its wheels at the start of the runway
    FixedWingState x = FixedWingState::Zero();
    x[fw::PN] = RUNWAY.north + 5.0;
    x[fw::PE] = RUNWAY.east;
    x[fw::PD] = -p.gear.front().position[2];
    x[fw::PSI] = RUNWAY.heading;
    FixedWingDynamics aircraft(p, x);
    aircraft.set_wind({options.wind_north, options.wind_east, 0.0});
    for (int k = 0; k < 100; ++k)
    {
        aircraft.step({}, STEP);
    }

#ifdef FIXED_WING_RENDER
    std::optional<Display> display;
    if (options.render)
    {
        display.emplace(mission, p, options.record);
    }
#else
    if (options.render)
    {
        std::cerr << "--render and --record need bitrl built with Chrono and Irrlicht\n";
        return 1;
    }
#endif

    std::cout << "    t [s]  phase         leg  north [m]  east [m]  alt [m]  Va [m/s]  "
                 "bank [deg]  off the line [m]\n";
    const auto print = [&](const std::string &note)
    {
        const auto &s = aircraft.state();
        std::cout << std::fixed << std::setprecision(1) << std::setw(9) << aircraft.time() << "  "
                  << std::left << std::setw(12) << to_string(mission.phase()) << std::right
                  << std::setw(5) << mission.leg() << std::setw(11) << s[fw::PN] << std::setw(10)
                  << s[fw::PE] << std::setw(9) << aircraft.altitude() << std::setw(10)
                  << aircraft.airspeed() << std::setw(12) << s[fw::PHI] / DEG << std::setw(18)
                  << mission.cross_track() << note << "\n";
    };

    MissionPhase phase = mission.phase();
    std::optional<real_t> liftoff_distance;
    real_t liftoff_airspeed = 0.0;
    real_t max_approach_off = 0.0;
    real_t max_glide_off = 0.0;
    real_t next_print = 0.0;
    real_t next_frame = 0.0;
    const real_t start_north = aircraft.state()[fw::PN];
    const auto wall_start = std::chrono::steady_clock::now();
    while (aircraft.time() < TIME_LIMIT && mission.phase() != MissionPhase::STOPPED)
    {
        const auto u = mission.update(aircraft, STEP);
        aircraft.step(u, STEP);

        if (!liftoff_distance && mission.phase() == MissionPhase::ROTATE && !aircraft.on_ground())
        {
            liftoff_distance = aircraft.state()[fw::PN] - start_north;
            liftoff_airspeed = aircraft.airspeed();
        }
        if (mission.phase() == MissionPhase::APPROACH)
        {
            const auto rc =
                mission.runway_coordinates(aircraft.state()[fw::PN], aircraft.state()[fw::PE]);
            // The last 300 m before the threshold, once lined up
            if (rc[0] > -300.0)
            {
                max_approach_off = std::max(max_approach_off, std::abs(mission.cross_track()));
                max_glide_off = std::max(
                    max_glide_off, std::abs(aircraft.altitude() - mission.commands().altitude));
            }
        }
        if (mission.phase() != phase)
        {
            phase = mission.phase();
            print("  <- " + to_string(phase));
        }
        else if (aircraft.time() + 1e-9 >= next_print)
        {
            print("");
        }
        if (aircraft.time() + 1e-9 >= next_print)
        {
            next_print += PRINT_PERIOD;
        }

#ifdef FIXED_WING_RENDER
        if (display && aircraft.time() + 1e-9 >= next_frame)
        {
            next_frame += 1.0 / RENDER_RATE;
            std::ostringstream status;
            status << std::fixed << std::setprecision(1) << "t = " << aircraft.time() << " s   "
                   << to_string(mission.phase()) << "   altitude " << aircraft.altitude()
                   << " m   airspeed " << aircraft.airspeed() << " m/s   bank "
                   << aircraft.state()[fw::PHI] / DEG << " deg   throttle " << std::setprecision(2)
                   << u.throttle << "   leg " << mission.leg() << "   yellow: flown, cyan: the "
                   << "line it follows";
            // Record the takeoff to 25 m, and the landing from 25 m
            const bool climbing_out = mission.phase() == MissionPhase::ENROUTE &&
                                      mission.leg() == 0 && aircraft.altitude() < 25.0;
            const bool landing =
                (mission.phase() == MissionPhase::APPROACH && aircraft.altitude() < 25.0) ||
                mission.phase() == MissionPhase::FLARE || mission.phase() == MissionPhase::ROLLOUT;
            const bool record = mission.phase() == MissionPhase::TAKEOFF_ROLL ||
                                mission.phase() == MissionPhase::ROTATE || climbing_out || landing;
            if (!display->show(aircraft, mission, status.str(), record))
            {
                return 0;
            }
            const real_t ahead =
                aircraft.time() -
                std::chrono::duration<real_t>(std::chrono::steady_clock::now() - wall_start)
                    .count();
            if (ahead > 0.0)
            {
                std::this_thread::sleep_for(std::chrono::duration<real_t>(ahead));
            }
        }
#else
        (void)next_frame;
#endif
    }

    const auto &s = aircraft.state();
    const auto end = mission.runway_coordinates(s[fw::PN], s[fw::PE]);
    std::cout << std::fixed << std::setprecision(1) << "\n";
    if (liftoff_distance)
    {
        std::cout << "  Lift-off after a roll of " << *liftoff_distance << " m, at "
                  << liftoff_airspeed << " m/s\n";
    }
    std::cout << std::setprecision(2) << "  On the last 300 m of the approach: at most "
              << max_approach_off << " m off the centre line and " << max_glide_off
              << " m off the glide slope\n";
    const auto &td = mission.touchdown();
    if (td)
    {
        std::cout << std::setprecision(1) << "  Touchdown at t = " << td->time << " s, "
                  << td->along << " m past the threshold, " << std::setprecision(2) << td->across
                  << " m off the centre line, sinking at " << td->sink_rate << " m/s, at "
                  << std::setprecision(1) << td->airspeed << " m/s\n";
    }
    std::cout << "  " << (mission.phase() == MissionPhase::STOPPED ? "Stopped" : "Still going")
              << " at t = " << aircraft.time() << " s, " << end[0] << " m past the threshold, "
              << std::setprecision(2) << end[1] << " m off the centre line\n";

    // Landed and stopped on the runway, and not too hard
    const bool on_runway =
        end[0] >= 0.0 && end[0] <= RUNWAY.length && std::abs(end[1]) < 0.5 * RUNWAY.width;
    return mission.phase() == MissionPhase::STOPPED && on_runway && td && td->sink_rate < 1.5 ? 0
                                                                                              : 1;
}

} // namespace fixed_wing_flight

int main(int argc, char **argv)
{
    using namespace fixed_wing_flight;

    Options options;
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "--wind" && i + 2 < argc)
        {
            options.wind_north = std::atof(argv[++i]);
            options.wind_east = std::atof(argv[++i]);
        }
        else if (arg == "--render")
        {
            options.render = true;
        }
        else if (arg == "--record" && i + 1 < argc)
        {
            options.render = true;
            options.record = argv[++i];
        }
        else
        {
            std::cerr << "Usage: fixed_wing_flight [--wind N E] [--render] [--record DIR]\n";
            return 1;
        }
    }

    const auto p = FixedWingParameters::aerosonde();
    show_trims(p);
    show_modes(p);
    FixedWingAutopilotDesign design;
    design.airspeed = DESIGN_AIRSPEED;
    FixedWingAutopilot autopilot(p, design);
    show_gains(autopilot);
    return fly(p, autopilot, options);
}

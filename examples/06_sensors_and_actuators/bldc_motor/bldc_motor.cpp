#include "bitsim/bitsim_config.h"

#ifdef BITSIM_CHRONO

#include "bitsim/actuators/bldc_motor.h"
#include "bitsim/bitsim_consts.h"
#include "bitsim/bitsim_types.h"
#include "bitsim/control/pi_controller.h"
#include "bitsim/utils/io/print_utils.h"

#include <chrono/core/ChFrame.h>
#include <chrono/core/ChMatrix33.h>
#include <chrono/functions/ChFunctionConst.h>
#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChLinkMotorRotationTorque.h>
#include <chrono/physics/ChSystemNSC.h>

#ifdef BITSIM_IRRLICHT
#include "bitsim/utils/render/irrlicht_utils.h"
#include <chrono/core/ChRealtimeStep.h>
#include <chrono_irrlicht/ChIrrTools.h>
#include <chrono_irrlicht/ChVisualSystemIrrlicht.h>
#endif

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace bldc_motor
{

using bitsim::real_t;
using chrono::ChFrame;
using chrono::ChMatrix33d;
using chrono::ChVector3d;

const real_t DEG = chrono::CH_DEG_TO_RAD;
const real_t PI = bitsim::consts::maths::PI;
const real_t RPM = 60.0 / (2.0 * PI); // rad/s to revolutions per minute

// ---------------------------------------------------------------------------
// The motor
// ---------------------------------------------------------------------------

// The motor, its DC-equivalent model, the three-phase model with six-step commutation and a
// simple rotor come from bitsim, see bitsim::actuators::BLDCMotor. The defaults of
// BLDCMotorParameters are a small brushless motor of the size used in robot joints
using namespace bitsim::actuators;
using bitsim::control::PIController;

// The three phases are called a, b and c
const std::array<char, 3> PHASE_NAMES{'a', 'b', 'c'};

// ---------------------------------------------------------------------------
// Printing
// ---------------------------------------------------------------------------
using bitsim::utils::io::print_title;
using bitsim::utils::io::print_value;

real_t clean(real_t x) { return std::abs(x) < 5.0e-4 ? 0.0 : x; }

// ---------------------------------------------------------------------------
// The sections of the lesson
// ---------------------------------------------------------------------------
void motor_constants(const BLDCMotorParameters &m)
{
    print_title("1. What the motor constants tell us");

    const real_t w_no_load = m.Kt() * m.V_supply / (m.R * m.b + m.Kt() * m.Ke);
    const real_t i_stall = m.V_supply / m.R;

    print_value("no-load speed", w_no_load, "rad/s");
    print_value("no-load speed", w_no_load * RPM, "rpm", 0);
    print_value("stall current", i_stall, "A", 1);
    print_value("stall torque", m.Kt() * i_stall, "N m");
    print_value("electrical time constant L/R", 1000.0 * m.L / m.R, "ms");
    print_value("mechanical time constant", 1000.0 * m.J * m.R / (m.Kt() * m.Ke), "ms");

    // Neglecting friction, the torque falls linearly with the speed
    std::cout << "\n    speed    torque   current     power\n"
              << "  (rad/s)     (N m)       (A)       (W)\n";
    for (const real_t fraction : {0.0, 0.25, 0.5, 0.75, 1.0})
    {
        const real_t w = fraction * m.V_supply / m.Ke;
        const real_t i = (m.V_supply - m.Ke * w) / m.R;
        const real_t T = m.Kt() * i;
        std::cout << "  " << std::fixed << std::setprecision(1) << std::setw(7) << w
                  << std::setprecision(3) << std::setw(10) << clean(T) << std::setprecision(1)
                  << std::setw(10) << clean(i) << std::setw(10) << clean(T * w) << "\n";
    }
}

void step_response(const BLDCMotorParameters &m)
{
    print_title("2. Switching the motor on");

    // The fastest time constant is 0.67 ms, so a step of 1 us is plenty for Euler
    const real_t dt = 1.0e-6;
    const real_t end_time = 0.05;
    const std::vector<real_t> report_times{0.0002, 0.0005, 0.001, 0.002, 0.005, 0.01, 0.02, 0.05};

    DCEquivalentState s;
    real_t peak_current = 0.0;
    real_t time_to_63 = -1.0;
    const real_t w_final = m.Kt() * m.V_supply / (m.R * m.b + m.Kt() * m.Ke);

    std::cout << "   t (ms)   current (A)   speed (rad/s)   torque (N m)\n";
    std::size_t next_report = 0;
    const int n_steps = static_cast<int>(std::round(end_time / dt));
    for (int n = 1; n <= n_steps; ++n)
    {
        s = dc_equivalent_step(m, s, m.V_supply, 0.0, dt);
        const real_t t = n * dt;
        peak_current = std::max(peak_current, s.i);
        if (time_to_63 < 0.0 && s.w >= 0.632 * w_final)
        {
            time_to_63 = t;
        }

        if (next_report < report_times.size() && t >= report_times[next_report] - dt / 2.0)
        {
            std::cout << "  " << std::fixed << std::setprecision(1) << std::setw(7) << 1000.0 * t
                      << std::setprecision(3) << std::setw(14) << s.i << std::setprecision(1)
                      << std::setw(16) << s.w << std::setprecision(3) << std::setw(15)
                      << m.Kt() * s.i << "\n";
            ++next_report;
        }
    }

    std::cout << "\n";
    print_value("peak current", peak_current, "A", 1);
    print_value("time to 63% of final speed", 1000.0 * time_to_63, "ms");
    print_value("final speed, simulated", s.w, "rad/s");
    print_value("final speed, predicted", w_final, "rad/s");
}

void commutation_table()
{
    print_title("3. Electronic commutation");

    std::cout << "  angle (el. deg)   Hall code   current flows\n";
    for (int angle = 0; angle < 360; angle += 30)
    {
        // Sample just after the angle, since the sector changes exactly at 30, 90, ...
        const int code = BLDCMotor::hall_code((angle + 1.0) * DEG);
        const PhasePair pair = BLDCMotor::commutation(code);
        std::cout << "  " << std::setw(15) << angle << std::setw(12) << code << "   "
                  << PHASE_NAMES[pair.high] << " to " << PHASE_NAMES[pair.low] << "\n";
    }
}

struct SteadyState
{
    real_t mean_speed;
    real_t mean_current;
    real_t mean_torque;
    real_t min_torque;
    real_t max_torque;
};

// Run the three-phase model at full voltage with a load, and average over the
// last 20 ms
SteadyState run_three_phase(const BLDCMotorParameters &m, real_t T_load, real_t hall_offset)
{
    BLDCMotor motor(m, hall_offset);
    BLDCRotor rotor;

    const real_t dt = 1.0e-6;
    const int n_steps = 100000;
    const int n_average = 20000;

    SteadyState result{0.0, 0.0, 0.0, 1.0e9, -1.0e9};
    for (int n = 0; n < n_steps; ++n)
    {
        const real_t T = motor.step(m.V_supply, rotor.theta, rotor.w, dt);
        rotor.step(m, T, T_load, dt);

        if (n >= n_steps - n_average)
        {
            result.mean_speed += rotor.w / n_average;
            result.mean_current += motor.current() / n_average;
            result.mean_torque += T / n_average;
            result.min_torque = std::min(result.min_torque, T);
            result.max_torque = std::max(result.max_torque, T);
        }
    }
    return result;
}

void three_phase_model(const BLDCMotorParameters &m)
{
    print_title("4. The three-phase model");

    // The DC model with the same load, at steady state
    const real_t T_load = 0.5;
    const real_t i_dc = (m.b * m.V_supply / m.Ke + T_load) / (m.Kt() + m.b * m.R / m.Ke);
    const real_t w_dc = (m.V_supply - m.R * i_dc) / m.Ke;

    const SteadyState s = run_three_phase(m, T_load, 0.0);

    std::cout << "  with a load of " << T_load << " N m      three-phase     DC model\n";
    std::cout << std::fixed << std::setprecision(1) << "  mean speed (rad/s)          "
              << std::setw(12) << s.mean_speed << std::setw(13) << w_dc << "\n"
              << std::setprecision(2) << "  mean current (A)            " << std::setw(12)
              << s.mean_current << std::setw(13) << i_dc << "\n"
              << std::setprecision(3) << "  torque min / mean / max     " << s.min_torque << " / "
              << s.mean_torque << " / " << s.max_torque << "\n";
    print_value("torque ripple (max-min)/mean", (s.max_torque - s.min_torque) / s.mean_torque, "");

    // The Hall sensors have to be in the right place. With an error the motor
    // commutates too early or too late. The torque per ampere is measured
    // with the rotor turned slowly, at a constant speed, so that the current
    // hardly changes
    std::cout << "\n  sensor error (el. deg)   torque per ampere (N m/A)   cos(error)\n";
    for (const real_t error : {0.0, 15.0, 30.0, 60.0, 90.0, 180.0})
    {
        BLDCMotor motor(m, error * DEG);
        const real_t w = 1.0;
        const real_t dt = 1.0e-5;
        real_t torque = 0.0;
        real_t current = 0.0;

        // one full electrical turn, after letting the current settle
        const int n_settle = 1000;
        const int n_turn = static_cast<int>(2.0 * PI / (m.pole_pairs * w) / dt);
        for (int n = 0; n < n_settle + n_turn; ++n)
        {
            const real_t T = motor.step(m.V_supply / 10.0, n * dt * w, w, dt);
            if (n >= n_settle)
            {
                torque += T / n_turn;
                current += motor.current() / n_turn;
            }
        }

        std::cout << "  " << std::fixed << std::setprecision(0) << std::setw(22) << error
                  << std::setprecision(4) << std::setw(28) << clean(torque / current)
                  << std::setw(13) << std::cos(error * DEG) << "\n";
    }
}

void speed_control(const BLDCMotorParameters &m)
{
    print_title("5. Speed control");

    BLDCMotor motor(m);
    BLDCRotor rotor;

    // Tuned on the DC model, where w/V = (1/Ke) / (tau_m s + 1). The integral
    // term cancels the mechanical time constant, and the closed loop responds
    // with a time constant of 1 ms
    const real_t tau_m = m.J * m.R / (m.Kt() * m.Ke);
    const real_t tau_closed_loop = 0.001;
    const real_t kp = m.Ke * tau_m / tau_closed_loop;
    PIController controller(kp, kp / tau_m, m.V_supply);

    // At a steady speed w most of the voltage only balances the back-EMF Ke w.
    // Feeding it forward leaves the PI controller to correct the rest
    const bool use_feedforward = true;

    // The controller runs at 10 kHz, the motor model at 1 MHz
    const real_t dt = 1.0e-6;
    const int steps_per_control = 100;

    const auto setpoint = [](real_t t) { return t < 0.08 ? 300.0 : -300.0; };
    const auto load = [](real_t t) { return t >= 0.04 && t < 0.08 ? 0.3 : 0.0; };

    real_t V = 0.0;
    real_t max_error_before_load = 0.0;
    real_t mean_current = 0.0; // over the last millisecond, the current ripples

    std::cout << "   t (ms)   setpoint   speed (rad/s)   voltage (V)   current (A)   load (N m)\n";
    const int n_steps = 160000;
    for (int n = 0; n < n_steps; ++n)
    {
        const real_t t = n * dt;
        if (n % steps_per_control == 0)
        {
            const real_t feedforward = use_feedforward ? m.Ke * setpoint(t) : 0.0;
            V = controller.update(setpoint(t) - rotor.w, steps_per_control * dt, feedforward);
        }

        const real_t T = motor.step(V, rotor.theta, rotor.w, dt);
        rotor.step(m, T, load(t), dt);
        mean_current += (motor.current() - mean_current) * dt / 0.001;

        if (t > 0.02 && t < 0.04)
        {
            max_error_before_load =
                std::max(max_error_before_load, std::abs(setpoint(t) - rotor.w));
        }

        if ((n + 1) % 10000 == 0)
        {
            std::cout << "  " << std::fixed << std::setprecision(0) << std::setw(7)
                      << 1000.0 * (t + dt) << std::setw(11) << setpoint(t) << std::setprecision(1)
                      << std::setw(16) << rotor.w << std::setw(14) << V << std::setprecision(2)
                      << std::setw(14) << mean_current << std::setw(13) << load(t) << "\n";
        }
    }

    std::cout << "\n";
    print_value("controller gain kp", kp, "V s/rad", 4);
    print_value("speed error from 20 to 40 ms", max_error_before_load, "rad/s", 2);
}

// The joint: the motor turns the shoulder of the arm of the forward_kinematics example through a
// gearbox. The link is the upper arm alone
struct Joint
{
    real_t gear_ratio = 20.0;
    real_t length = 0.4;
    real_t mass = 1.0;
};

// The upper arm in Chrono, driven by our motor model through a gearbox, with
// a PI position controller. Chrono steps every millisecond and the motor 100
// times in between
class MotorisedJoint
{
  public:
    static constexpr real_t STEP_SIZE = 0.001; // Chrono and the controller
    static constexpr int MOTOR_SUBSTEPS = 100; // the motor model runs 100 times faster

    MotorisedJoint(const BLDCMotorParameters &m, const Joint &joint, real_t ki)
        : m_(m), joint_(joint), motor_(m),
          // A PI position controller: the voltage is proportional to the angle
          // error and its integral. The back-EMF already damps the motion, so we
          // do not need a derivative term
          controller_(20.0, ki, m.V_supply)
    {
        system_.SetGravitationalAcceleration(ChVector3d(0.0, 0.0, -9.81));
        system_.SetSolverType(chrono::ChSolver::Type::SPARSE_QR);

        auto base = chrono_types::make_shared<chrono::ChBody>();
        base->SetFixed(true);
        system_.Add(base);

        const real_t width = 0.04;
        const real_t density = joint.mass / (joint.length * width * width);
        link_ = chrono_types::make_shared<chrono::ChBodyEasyBox>(joint.length, width, width,
                                                                 density, true, false);
        link_->SetCoordsys(
            (X_W1 * ChFrame<>(ChVector3d(joint.length / 2.0, 0.0, 0.0))).GetCoordsys());

        // The rotor turns N times faster than the link, so it adds N^2 J to the
        // inertia around the joint axis, which is the z axis of the link
        ChVector3d inertia = link_->GetInertiaXX();
        inertia.z() += joint.gear_ratio * joint.gear_ratio * m.J;
        link_->SetInertiaXX(inertia);
        system_.Add(link_);

        torque_ = chrono_types::make_shared<chrono::ChFunctionConst>(0.0);
        actuator_ = chrono_types::make_shared<chrono::ChLinkMotorRotationTorque>();
        actuator_->Initialize(link_, base, X_W1);
        actuator_->SetTorqueFunction(torque_);
        system_.Add(actuator_);
    }

    // One step of the controller, the motor and Chrono
    void step(real_t target)
    {
        const real_t q = angle();
        const real_t q_dot = actuator_->GetMotorAngleDt();
        voltage_ = controller_.update(target - q, STEP_SIZE);

        // The rotor follows the link through the gearbox. Over one Chrono step
        // we hold its speed and advance the current, and average the torque
        const real_t dt = STEP_SIZE / MOTOR_SUBSTEPS;
        real_t motor_torque = 0.0;
        for (int k = 0; k < MOTOR_SUBSTEPS; ++k)
        {
            const real_t theta = joint_.gear_ratio * (q + k * dt * q_dot);
            const real_t w = joint_.gear_ratio * q_dot;
            motor_torque += (motor_.step(voltage_, theta, w, dt) - m_.b * w) / MOTOR_SUBSTEPS;
        }

        // The gearbox multiplies the torque by N
        joint_torque_ = joint_.gear_ratio * motor_torque;
        torque_->SetConstant(joint_torque_);
        system_.DoStepDynamics(STEP_SIZE);
    }

    real_t time() const { return system_.GetChTime(); }
    real_t angle() const { return actuator_->GetMotorAngle(); }
    real_t voltage() const { return voltage_; }
    real_t joint_torque() const { return joint_torque_; }
    const BLDCMotor &motor() const { return motor_; }

    // The electrical angle of the rotor
    real_t electrical_angle() const { return m_.pole_pairs * joint_.gear_ratio * angle(); }

    chrono::ChSystemNSC &system() { return system_; }
    std::shared_ptr<chrono::ChBody> link() const { return link_; }

    // The joint frame of the shoulder in the forward_kinematics example: its z axis is horizontal
    inline static const ChFrame<> X_W1{ChVector3d(0.0, 0.0, 0.3),
                                       ChMatrix33d(90.0 * DEG, ChVector3d(1.0, 0.0, 0.0))};

  private:
    BLDCMotorParameters m_;
    Joint joint_;
    BLDCMotor motor_;
    PIController controller_;
    chrono::ChSystemNSC system_;
    std::shared_ptr<chrono::ChBody> link_;
    std::shared_ptr<chrono::ChFunctionConst> torque_;
    std::shared_ptr<chrono::ChLinkMotorRotationTorque> actuator_;
    real_t voltage_ = 0.0;
    real_t joint_torque_ = 0.0;
};

struct HoldResult
{
    real_t final_angle;
    real_t final_current;
};

HoldResult lift_the_arm(const BLDCMotorParameters &m, const Joint &joint, real_t target, real_t ki,
                        bool verbose)
{
    MotorisedJoint model(m, joint, ki);

    if (verbose)
    {
        std::cout << "   t (s)   angle (deg)   voltage (V)   current (A)   joint torque (N m)\n";
    }

    const real_t step_size = MotorisedJoint::STEP_SIZE;
    int step = 0;
    while (model.time() < 2.0 - step_size / 2.0)
    {
        model.step(target);
        ++step;

        if (verbose && step % 200 == 0)
        {
            std::cout << "  " << std::fixed << std::setprecision(1) << std::setw(6) << model.time()
                      << std::setw(14) << model.angle() / DEG << std::setprecision(2)
                      << std::setw(14) << model.voltage() << std::setw(14)
                      << model.motor().current() << std::setprecision(3) << std::setw(21)
                      << model.joint_torque() << "\n";
        }
    }

    return {model.angle(), model.motor().current()};
}

void motor_in_a_joint(const BLDCMotorParameters &m)
{
    print_title("6. A motor in the joint of an arm");

    const Joint joint;
    const real_t target = 45.0 * DEG;

    std::cout << "  PI controller\n";
    const HoldResult pi = lift_the_arm(m, joint, target, 100.0, true);

    std::cout << "\n  P controller, after 2 s\n";
    const HoldResult p = lift_the_arm(m, joint, target, 0.0, false);
    print_value("angle", p.final_angle / DEG, "deg", 2);
    print_value("current", p.final_current, "A", 2);

    // To hold the arm still the motor has to balance the weight of the link
    const real_t gravity_torque = joint.mass * 9.81 * joint.length / 2.0 * std::cos(pi.final_angle);
    const real_t motor_torque = gravity_torque / joint.gear_ratio;
    BLDCMotor motor(m);

    std::cout << "\n  Holding the arm at " << std::fixed << std::setprecision(2)
              << pi.final_angle / DEG << " deg\n";
    print_value("torque of gravity at the joint", gravity_torque, "N m");
    print_value("torque at the motor", motor_torque, "N m", 4);
    print_value("current needed, at least", motor_torque / motor.k_line(), "A");
    print_value("current needed, at most", motor_torque / (motor.k_line() * std::cos(30.0 * DEG)),
                "A");
    print_value("current in the simulation", pi.final_current, "A");
}

#ifdef BITSIM_IRRLICHT
// A circle in the xy plane of the given frame
void draw_ring(chrono::irrlicht::ChVisualSystemIrrlicht &vis, const ChFrame<> &frame, real_t radius,
               const chrono::ChColor &color)
{
    const int n_segments = 48;
    const auto point = [&](int i)
    {
        const real_t angle = 2.0 * PI * i / n_segments;
        return frame * ChVector3d(radius * std::cos(angle), radius * std::sin(angle), 0.0);
    };
    for (int i = 0; i < n_segments; ++i)
    {
        bitsim::utils::render::irr_draw_line(vis, point(i), point(i + 1), color, 2.0);
    }
}

// A schematic of one electrical turn of the motor. The arrow is the electrical
// angle of the rotor. The phase the current goes into is red, the one it comes
// out of is blue, and the disconnected one is grey
void draw_motor(chrono::irrlicht::ChVisualSystemIrrlicht &vis, const ChFrame<> &frame,
                real_t theta_e, const PhasePair &pair)
{
    using namespace bitsim::utils::render;

    const real_t radius = 0.15;
    draw_ring(vis, frame, radius, chrono::ChColor(0.6f, 0.6f, 0.6f));

    for (int phase = 0; phase < 3; ++phase)
    {
        const real_t angle = phase * 120.0 * DEG;
        const ChVector3d direction(std::cos(angle), std::sin(angle), 0.0);
        chrono::ChColor color(0.5f, 0.5f, 0.5f);
        if (phase == pair.high)
        {
            color = chrono::ChColor(1.0f, 0.2f, 0.2f);
        }
        else if (phase == pair.low)
        {
            color = chrono::ChColor(0.2f, 0.5f, 1.0f);
        }
        irr_draw_line(vis, frame * (direction * (radius * 0.75)),
                      frame * (direction * radius * 1.1), color, 8.0);
        irr_draw_label(vis, std::string(1, PHASE_NAMES[phase]),
                       frame * (direction * radius * 1.35));
    }

    const ChVector3d rotor(std::cos(theta_e), std::sin(theta_e), 0.0);
    irr_draw_line(vis, frame.GetPos(), frame * (rotor * (radius * 0.65)),
                  chrono::ChColor(1.0f, 0.8f, 0.2f), 4.0);
    irr_draw_label(vis, "motor, one electrical turn",
                   frame * ChVector3d(-radius, -radius * 1.5, 0.0));
}

// The arm of section 6 goes from one target to the next, in slow motion
void render(const BLDCMotorParameters &m)
{
    using namespace bitsim::utils::render;

    const Joint joint;
    MotorisedJoint model(m, joint, 100.0);
    model.link()->GetVisualShape(0)->SetColor(chrono::ChColor(0.55f, 0.70f, 0.90f));

    chrono::irrlicht::ChVisualSystemIrrlicht vis;
    vis.AttachSystem(&model.system());
    vis.SetCameraVertical(chrono::CameraVerticalDir::Z);
    vis.SetWindowSize(1280, 720);
    vis.SetWindowTitle("A brushless motor in a joint");
    vis.Initialize();
    vis.AddCamera({-0.1, -1.6, 0.45}, {-0.1, 0.0, 0.35});
    vis.GetActiveCamera()->setFOV(0.75f);
    vis.AddTypicalLights();
    vis.BindAll();

    auto *readout = vis.GetGUIEnvironment()->addStaticText(
        L"", irr::core::rect<irr::s32>(1020, 10, 1270, 160), true, true);
    readout->setBackgroundColor(irr::video::SColor(220, 255, 255, 255));
    readout->setDrawBackground(true);

    // A new target every 2.5 s
    const std::array<real_t, 5> targets{45.0 * DEG, 90.0 * DEG, 0.0, -45.0 * DEG, 135.0 * DEG};
    const real_t hold_time = 2.5;

    // The motor is too fast to follow in real time. At full speed the rotor
    // makes more than 15 electrical turns per second
    const real_t slow_motion = 4.0;
    const int steps_per_frame = 10;
    chrono::ChRealtimeStepTimer real_time;

    // The schematic of the motor sits next to the arm, in the plane of the motion
    const ChFrame<> motor_frame(ChVector3d(-0.55, 0.0, 0.45),
                                ChMatrix33d(90.0 * DEG, ChVector3d(1.0, 0.0, 0.0)));

    while (vis.Run())
    {
        const std::size_t index =
            static_cast<std::size_t>(model.time() / hold_time) % targets.size();
        const real_t target = targets[index];
        const BLDCMotor &motor = model.motor();

        std::ostringstream text;
        text << "t = " << std::fixed << std::setprecision(2) << model.time() << " s"
             << "  (" << std::setprecision(0) << slow_motion << "x slower)\n\n"
             << std::setprecision(1) << "target = " << target / DEG << " deg\n"
             << "angle = " << model.angle() / DEG << " deg\n\n"
             << std::setprecision(2) << "voltage = " << model.voltage() << " V\n"
             << "current = " << motor.current() << " A\n"
             << "joint torque = " << model.joint_torque() << " N m\n\n"
             << "current flows " << PHASE_NAMES[motor.pair().high] << " to "
             << PHASE_NAMES[motor.pair().low] << "\n";
        readout->setText(irr::core::stringw(text.str().c_str()).c_str());

        vis.BeginScene();
        vis.Render();

        chrono::irrlicht::tools::drawGrid(&vis, 0.25, 0.25, 8, 8, chrono::ChCoordsysd(),
                                          chrono::ChColor(0.35f, 0.45f, 0.55f));

        // The joint turns around the blue axis. The white line is the target
        const ChFrame<> &X_W1 = MotorisedJoint::X_W1;
        irr_draw_frame(vis, X_W1, 0.1);
        const ChVector3d to_target = X_W1.TransformDirectionLocalToParent(
            ChVector3d(std::cos(target), std::sin(target), 0.0));
        irr_draw_line(vis, X_W1.GetPos(), X_W1.GetPos() + to_target * (joint.length + 0.1),
                      chrono::ChColor(1.0f, 1.0f, 1.0f), 1.0);

        draw_motor(vis, motor_frame, model.electrical_angle(), motor.pair());

        for (int i = 0; i < steps_per_frame; ++i)
        {
            model.step(target);
        }
        vis.EndScene();
        real_time.Spin(slow_motion * steps_per_frame * MotorisedJoint::STEP_SIZE);
    }
}
#endif

} // namespace bldc_motor

int main(int argc, char **argv)
{
    using namespace bldc_motor;

    const BLDCMotorParameters motor;

    motor_constants(motor);
    step_response(motor);
    commutation_table();
    three_phase_model(motor);
    speed_control(motor);
    motor_in_a_joint(motor);

    if (argc > 1 && std::string(argv[1]) == "--render")
    {
#ifdef BITSIM_IRRLICHT
        render(motor);
#else
        std::cerr << "--render needs bitsim configured with BITSIM_IRRLICHT=ON" << std::endl;
        return 1;
#endif
    }

    return 0;
}

#else
#include <iostream>
int main()
{
    std::cerr << "You need PROJECTCHRONO configured with "
              << "bitsim in order to run this example "
              << "Reconfigure bitsim and set ENABLE_CHRONO=ON" << std::endl;
    return 1;
}
#endif

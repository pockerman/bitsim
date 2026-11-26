// Balancing a cart-pole near the end of its track, with an LQR and with model predictive control.
//
// Usage: cart_pole_lqr_mpc [--record DIR]
//
// Both controllers start with the pole leaning 0.4 rad towards the near end of the track, 0.9 m
// from it, and can push the cart with at most 10 N. The LQR is designed without the limits, and
// its force is clipped to them. MPC plans with the limits of the force and of the track. --record
// DIR writes a picture of both every 0.04 s of simulated time to DIR, which needs bitsim built
// with OpenCV.
#include "bitsim/bitsim_config.h"
#include "bitsim/bitsim_types.h"
#include "bitsim/control/linear_mpc.h"
#include "bitsim/control/lqr.h"
#include "bitsim/dynamics/cart_pole_dynamics.h"

#ifdef BITSIM_OPENCV
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#endif

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace cart_pole_lqr_mpc
{

using bitsim::real_t;
using bitsim::RealColVec4d;
using bitsim::uint_t;
using bitsim::control::Matrix;
using bitsim::control::Vector;
using bitsim::dynamics::CartPoleDynamics;
using bitsim::dynamics::CartPoleParameters;
using bitsim::dynamics::CartPoleState;
namespace cp = bitsim::dynamics::cart_pole;

const real_t DT = 0.02;          // the control period [s]
const real_t DURATION = 6.0;     // [s]
const real_t X0 = 1.5;           // where the cart starts [m]
const real_t THETA0 = 0.4;       // how far the pole leans, towards +x [rad]
const real_t TRACK_BOUND = 2.3;  // what MPC keeps the cart within, 0.1 m short of the end [m]
const uint_t HORIZON = 40;       // the steps MPC plans, 0.8 s
const real_t PENALTY = 1000.0;   // of a cart beyond TRACK_BOUND
const real_t RECORD_RATE = 25.0; // [pictures/s]

// One run: the state at every step, and the force
struct Run
{
    std::string name;
    std::vector<CartPoleState> states;
    std::vector<real_t> forces;
    std::vector<std::vector<real_t>> plans; // MPC's planned cart positions, at every step
    real_t max_x{0.0};
    bool off_track{false};
};

// The weights of both controllers: the angle of the pole matters most, then the position
Matrix state_weights()
{
    RealColVec4d q;
    q << 1.0, 0.1, 10.0, 0.1;
    return q.asDiagonal().toDenseMatrix();
}

Matrix input_weights() { return 0.1 * Matrix::Identity(1, 1); }

CartPoleState start()
{
    CartPoleState x = CartPoleState::Zero();
    x[cp::X] = X0;
    x[cp::THETA] = THETA0;
    return x;
}

// Run a controller, a function from the state to the force, and record what happens
template <typename Controller> Run simulate(const std::string &name, Controller &&controller)
{
    CartPoleDynamics cart_pole(CartPoleParameters(), start());
    Run run;
    run.name = name;
    const auto steps = static_cast<uint_t>(std::lround(DURATION / DT));
    for (uint_t k = 0; k <= steps; ++k)
    {
        const CartPoleState x = cart_pole.state();
        const real_t force = cart_pole.limit(controller(x, run));
        run.states.push_back(x);
        run.forces.push_back(force);
        run.max_x = std::max(run.max_x, x[cp::X]);
        run.off_track = run.off_track || cart_pole.off_track();
        cart_pole.step(force, DT);
    }
    return run;
}

void print(const Run &run)
{
    std::printf("\n%s\n  time [s]   cart [m]   pole [rad]   force [N]\n", run.name.c_str());
    for (uint_t k = 0; k < run.states.size(); k += 25)
    {
        const auto &x = run.states[k];
        std::printf("  %8.2f   %8.3f   %10.3f   %9.2f\n", k * DT, x[cp::X], x[cp::THETA],
                    run.forces[k]);
    }
    std::printf("  furthest the cart went: %.3f m, %s\n", run.max_x,
                run.off_track ? "off the end of the track" : "on the track");
}

#ifdef BITSIM_OPENCV
// Draw both runs, one above the other, at step k
cv::Mat draw(const std::vector<Run> &runs, uint_t k, const CartPoleParameters &p)
{
    const int width = 640;
    const int panel = 250;
    cv::Mat picture(panel * static_cast<int>(runs.size()), width, CV_8UC3,
                    cv::Scalar(250, 250, 250));

    // the part of the track where it happens, from -0.5 m to 2.9 m
    const real_t view_min = -0.5;
    const real_t view_max = 2.9;
    const real_t scale = width / (view_max - view_min); // pixels per metre
    const auto px = [&](real_t x) { return static_cast<int>(std::lround((x - view_min) * scale)); };

    for (uint_t r = 0; r < runs.size(); ++r)
    {
        const Run &run = runs[r];
        const auto &x = run.states[std::min<uint_t>(k, run.states.size() - 1)];
        const int top = static_cast<int>(r) * panel;
        const int ground = top + panel - 40;

        // the track and MPC's bound; the end of the track is drawn over the cart
        cv::line(picture, {0, ground}, {px(p.track_half_length), ground}, cv::Scalar(90, 90, 90),
                 2);
        for (int y = ground - 200; y < ground; y += 8)
        {
            cv::line(picture, {px(TRACK_BOUND), y}, {px(TRACK_BOUND), y + 4},
                     cv::Scalar(120, 120, 220), 1);
        }

        // MPC's planned cart positions
        if (!run.plans.empty())
        {
            for (const real_t xp : run.plans[std::min<uint_t>(k, run.plans.size() - 1)])
            {
                cv::circle(picture, {px(xp), ground + 14}, 2, cv::Scalar(200, 150, 60), -1);
            }
        }

        // the cart, the pole and the force
        const int cart_x = px(x[cp::X]);
        const int cart_y = ground - 14;
        const cv::Scalar cart_colour = std::abs(x[cp::X]) > p.track_half_length
                                           ? cv::Scalar(60, 60, 220)
                                           : cv::Scalar(70, 70, 70);
        const int half = static_cast<int>(0.25 * scale); // a cart 0.5 m long
        cv::rectangle(picture, {cart_x - half, cart_y - 14}, {cart_x + half, cart_y + 14},
                      cart_colour, -1);
        const real_t length = 2.0 * p.pole_half_length * scale;
        const cv::Point tip(cart_x + static_cast<int>(length * std::sin(x[cp::THETA])),
                            cart_y - static_cast<int>(length * std::cos(x[cp::THETA])));
        cv::line(picture, {cart_x, cart_y}, tip, cv::Scalar(30, 140, 230), 6, cv::LINE_AA);
        cv::line(picture, {px(p.track_half_length), ground - 40},
                 {px(p.track_half_length), ground + 8}, cv::Scalar(40, 40, 200), 4);
        cv::putText(picture, "end", {px(p.track_half_length) + 6, ground - 30},
                    cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(40, 40, 200), 1, cv::LINE_AA);
        cv::putText(picture, "bound", {px(TRACK_BOUND) + 6, ground - 120}, cv::FONT_HERSHEY_SIMPLEX,
                    0.45, cv::Scalar(120, 120, 220), 1, cv::LINE_AA);
        const real_t force = run.forces[std::min<uint_t>(k, run.forces.size() - 1)];
        if (std::abs(force) > 0.2)
        {
            const int arrow = static_cast<int>(force * 6.0);
            cv::arrowedLine(picture, {cart_x, cart_y + 26}, {cart_x + arrow, cart_y + 26},
                            cv::Scalar(60, 160, 60), 2, cv::LINE_AA, 0, 0.3);
        }

        char text[120];
        std::snprintf(text, sizeof(text), "%s   t %.2f s   cart %.2f m   force %+.1f N",
                      run.name.c_str(), k * DT, x[cp::X], force);
        cv::putText(picture, text, {12, top + 24}, cv::FONT_HERSHEY_SIMPLEX, 0.55,
                    cv::Scalar(20, 20, 20), 1, cv::LINE_AA);
        if (r > 0)
        {
            cv::line(picture, {0, top}, {width, top}, cv::Scalar(200, 200, 200), 1);
        }
    }
    return picture;
}

void record(const std::vector<Run> &runs, const std::string &directory)
{
    std::filesystem::create_directories(directory);
    const auto every = static_cast<uint_t>(std::lround(1.0 / (RECORD_RATE * DT)));
    uint_t n = 0;
    for (uint_t k = 0; k < runs.front().states.size(); k += every)
    {
        char name[32];
        std::snprintf(name, sizeof(name), "/frame_%04u.png", static_cast<unsigned>(n++));
        cv::imwrite(directory + name, draw(runs, k, CartPoleParameters()));
    }
    std::cout << "Wrote " << n << " pictures to " << directory << std::endl;
}
#endif

} // namespace cart_pole_lqr_mpc

int main(int argc, char **argv)
{
    using namespace cart_pole_lqr_mpc;

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
            std::cerr << "Usage: cart_pole_lqr_mpc [--record DIR]\n";
            return 1;
        }
    }

    // the model, linearised about upright and discretised for the control period
    const CartPoleDynamics model;
    const auto linear = model.linearise();
    const auto discrete = bitsim::control::discretise(linear.A, linear.B, DT);
    const Matrix Q = state_weights();
    const Matrix R = input_weights();
    const real_t max_force = model.parameters().max_force;

    // the LQR, with its force clipped to the limit
    const auto lqr = bitsim::control::dlqr(discrete.A, discrete.B, Q, R);
    bitsim::control::LQRController lqr_controller(lqr.K);
    lqr_controller.set_input_bounds(Vector::Constant(1, -max_force),
                                    Vector::Constant(1, max_force));
    std::printf("LQR gain K = [%.2f %.2f %.2f %.2f]\n", lqr.K(0, 0), lqr.K(0, 1), lqr.K(0, 2),
                lqr.K(0, 3));

    // MPC, with the same weights, the limit of the force, and the track, softly
    const real_t inf = std::numeric_limits<real_t>::infinity();
    bitsim::control::LinearMPC mpc(4, 1, HORIZON);
    mpc.set_model(discrete.A, discrete.B);
    mpc.set_weights(Q, R);
    mpc.set_input_bounds(Vector::Constant(1, -max_force), Vector::Constant(1, max_force));
    Vector x_min(4);
    x_min << -TRACK_BOUND, -inf, -inf, -inf;
    Vector x_max(4);
    x_max << TRACK_BOUND, inf, inf, inf;
    mpc.set_state_bounds(x_min, x_max, PENALTY);

    const Run lqr_run = simulate("LQR", [&](const CartPoleState &x, Run &)
                                 { return lqr_controller.control(x)[0]; });

    uint_t unsolved = 0;
    uint_t iterations = 0;
    const Run mpc_run =
        simulate("MPC",
                 [&](const CartPoleState &x, Run &run)
                 {
                     const real_t force = mpc.solve(x)[0];
                     unsolved +=
                         mpc.status() == bitsim::control::LinearMPC::QPStatus::SOLVED ? 0 : 1;
                     iterations += mpc.iterations();
                     std::vector<real_t> plan;
                     for (const auto &xp : mpc.predicted_states())
                     {
                         plan.push_back(xp[cp::X]);
                     }
                     run.plans.push_back(plan);
                     return force;
                 });

    print(lqr_run);
    print(mpc_run);
    std::printf("\nMPC solved %zu QPs, %u of them not to the tolerance, with %.0f iterations on "
                "average\n",
                mpc_run.states.size(), static_cast<unsigned>(unsolved),
                static_cast<double>(iterations) / mpc_run.states.size());

    if (!record_dir.empty())
    {
#ifdef BITSIM_OPENCV
        record({lqr_run, mpc_run}, record_dir);
#else
        std::cerr << "--record needs bitsim built with OpenCV\n";
        return 1;
#endif
    }

    // what the example shows: MPC keeps the cart on the track and balances the pole
    const auto &last = mpc_run.states.back();
    const bool balanced = std::abs(last[cp::THETA]) < 0.01 && std::abs(last[cp::X]) < 0.1;
    return (!mpc_run.off_track && balanced) ? 0 : 1;
}

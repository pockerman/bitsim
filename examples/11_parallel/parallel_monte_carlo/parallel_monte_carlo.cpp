// How far off is dead reckoning after a lap? A Monte Carlo study on all the cores.
//
// A differential drive robot drives a 2 m square by odometry alone: its wheels slip a little,
// so the speed it moves at is not quite the one it commanded, and its turn rate has a small
// error too. Each run simulates one lap with its own random errors, and many runs give the
// distribution of where the robot ends up. The runs are independent, so the example runs them
// with parallel_map, first on one thread and then on all of them, and checks that the results
// are the same.
//
// Usage: parallel_monte_carlo [--runs N]
//
// Writes final_positions.csv, the end of every run, to the working directory.

#include "bitsim/bitsim_consts.h"
#include "bitsim/bitsim_types.h"
#include "bitsim/dynamics/diff_drive_dynamics.h"
#include "bitsim/dynamics/system_state.h"
#include "bitsim/utils/concurrency/parallel.h"
#include "bitsim/utils/io/print_utils.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <vector>

namespace parallel_monte_carlo
{

using bitsim::real_t;
using bitsim::uint_t;
using bitsim::dynamics::DiffDriveDynamics;
using bitsim::dynamics::SysState;
using namespace bitsim::utils::concurrency;

const real_t PI = bitsim::consts::maths::PI;
const real_t SIDE = 2.0;           // [m]
const real_t V = 0.5;              // [m/s]
const real_t W = PI / 2.0;         // [rad/s], a quarter turn in 1 s
const real_t DT = 0.01;            // [s]
const real_t SPEED_NOISE = 0.05;   // relative error of the speed, per step
const real_t RATE_NOISE = 0.05;    // error of the turn rate, per step [rad/s]
const real_t RATE_BIAS_STD = 0.01; // a bias of the turn rate for the whole lap [rad/s]
const uint_t SEED = 42;

struct End
{
    real_t x, y, theta;
};

// One lap of the square by odometry, with the errors of run `run`
End lap(uint_t run)
{
    std::mt19937 rng(static_cast<std::mt19937::result_type>(task_seed(SEED, run)));
    std::normal_distribution<real_t> speed_error(0.0, SPEED_NOISE);
    std::normal_distribution<real_t> rate_error(0.0, RATE_NOISE);
    const real_t bias = std::normal_distribution<real_t>(0.0, RATE_BIAS_STD)(rng);

    SysState<3> state(
        {std::make_pair("X", 0.0), std::make_pair("Y", 0.0), std::make_pair("Theta", 0.0)});
    const auto n_straight = static_cast<uint_t>(std::lround(SIDE / (V * DT)));
    const auto n_turn = static_cast<uint_t>(std::lround((PI / 2.0) / (W * DT)));
    for (uint_t side = 0; side < 4; ++side)
    {
        for (uint_t k = 0; k < n_straight; ++k)
        {
            const real_t v = V * (1.0 + speed_error(rng));
            state = DiffDriveDynamics::integrate_state_v2(state, DT, v, bias + rate_error(rng),
                                                          {0.0, 0.0});
        }
        for (uint_t k = 0; k < n_turn; ++k)
        {
            state = DiffDriveDynamics::integrate_state_v2(state, DT, 0.0,
                                                          W + bias + rate_error(rng), {0.0, 0.0});
        }
    }
    return {state[0], state[1], std::remainder(state[2], 2.0 * PI)};
}

struct Statistics
{
    real_t mean_error{0.0}, rms_error{0.0}, p95_error{0.0}, mean_x{0.0}, mean_y{0.0};
};

// Without errors the robot ends where it started, so the error is the distance from the start
Statistics statistics(const std::vector<End> &ends, std::size_t n)
{
    std::vector<real_t> errors(n);
    Statistics s;
    for (std::size_t i = 0; i < n; ++i)
    {
        errors[i] = std::hypot(ends[i].x, ends[i].y);
        s.mean_error += errors[i];
        s.rms_error += errors[i] * errors[i];
        s.mean_x += ends[i].x;
        s.mean_y += ends[i].y;
    }
    s.mean_error /= n;
    s.rms_error = std::sqrt(s.rms_error / n);
    s.mean_x /= n;
    s.mean_y /= n;
    std::sort(errors.begin(), errors.end());
    s.p95_error = errors[static_cast<std::size_t>(0.95 * (n - 1))];
    return s;
}

void run(uint_t n_runs)
{
    bitsim::utils::io::print_title("1. One lap without errors");
    {
        SysState<3> state(
            {std::make_pair("X", 0.0), std::make_pair("Y", 0.0), std::make_pair("Theta", 0.0)});
        for (uint_t side = 0; side < 4; ++side)
        {
            for (uint_t k = 0; k < 400; ++k)
            {
                state = DiffDriveDynamics::integrate_state_v2(state, DT, V, 0.0, {0.0, 0.0});
            }
            for (uint_t k = 0; k < 100; ++k)
            {
                state = DiffDriveDynamics::integrate_state_v2(state, DT, 0.0, W, {0.0, 0.0});
            }
        }
        std::cout << "  the robot ends " << std::hypot(state[0], state[1])
                  << " m from where it started: back at the start, but for rounding\n\n";
    }

    bitsim::utils::io::print_title("2. " + std::to_string(n_runs) + " laps with errors");
    using clock = std::chrono::steady_clock;

    // a plain loop, for the time of one thread
    auto start = clock::now();
    std::vector<End> serial;
    serial.reserve(n_runs);
    for (uint_t i = 0; i < n_runs; ++i)
    {
        serial.push_back(lap(i));
    }
    const std::chrono::duration<real_t> t_serial = clock::now() - start;

    start = clock::now();
    const auto parallel = parallel_map(n_runs, lap); // the default pool, every core
    const std::chrono::duration<real_t> t_parallel = clock::now() - start;

    bool same = true;
    for (std::size_t i = 0; i < n_runs; ++i)
    {
        same = same && serial[i].x == parallel[i].x && serial[i].y == parallel[i].y;
    }
    std::cout << std::fixed << std::setprecision(2) << "  on one thread: " << t_serial.count()
              << " s; on " << default_thread_pool().size()
              << " threads of the pool and this one: " << t_parallel.count() << " s, a speed-up of "
              << t_serial.count() / t_parallel.count() << "\n"
              << "  every run ended in the same place on both: " << std::boolalpha << same
              << "\n\n";

    bitsim::utils::io::print_title("3. Where the robot ends up");
    std::cout << "      runs   mean error [m]   rms error [m]   95 % within [m]   mean end [m]\n";
    // more runs, a better estimate: the error of a mean shrinks with the square root of the runs
    std::vector<std::size_t> sizes;
    for (std::size_t n = 100; n < n_runs; n *= 10)
    {
        sizes.push_back(n);
    }
    sizes.push_back(n_runs);
    for (const std::size_t n : sizes)
    {
        const auto s = statistics(parallel, n);
        std::cout << std::setw(10) << n << std::setprecision(3) << std::setw(17) << s.mean_error
                  << std::setw(16) << s.rms_error << std::setw(18) << s.p95_error << "   ("
                  << s.mean_x << ", " << s.mean_y << ")\n";
    }

    std::ofstream out("final_positions.csv");
    out << "x,y,theta\n";
    for (const auto &e : parallel)
    {
        out << e.x << "," << e.y << "," << e.theta << "\n";
    }
    std::cout << "\n  The ends of the runs are in final_positions.csv\n";
}

} // namespace parallel_monte_carlo

int main(int argc, char **argv)
{
    bitsim::uint_t n_runs = 20000;
    for (int i = 1; i + 1 < argc; i += 2)
    {
        if (std::string(argv[i]) == "--runs")
        {
            n_runs = std::strtoul(argv[i + 1], nullptr, 10);
        }
    }
    parallel_monte_carlo::run(std::max<bitsim::uint_t>(n_runs, 100));
    return 0;
}

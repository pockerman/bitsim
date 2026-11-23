// Two masses joined by a stiff spring, simulated as two subsystems that meet only every macro
// step: the benchmark of co-simulation. It compares loose coupling, Gauss-Seidel and strong
// coupling with the whole system integrated by one solver.
//
// Usage: two_mass_oscillator [--csv oscillator.csv]
#include "bitsim/bitsim_types.h"
#include "bitsim/cosim/co_simulation.h"
#include "bitsim/cosim/cosim_subsystem.h"
#include "bitsim/utils/concurrency/thread_pool.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

namespace
{
using bitsim::real_t;
using bitsim::uint_t;
using namespace bitsim::cosim;

// mass 1 on a soft spring, mass 2 on a stiff one, joined by a stiff spring with a little damping
const real_t M1 = 1.0, K1 = 10.0;
const real_t M2 = 1.0, K2 = 1000.0, D2 = 0.1;
const real_t KC = 100.0, DC = 0.1;
const uint_t SUBSTEPS = 10; // Runge-Kutta steps of each subsystem per macro step

typedef std::vector<real_t> vec;

// the force of the coupling spring on mass 1, from the state of mass 2 and that of mass 1
real_t coupling_force(const vec &x2, const vec &x1)
{
    return KC * (x2[0] - x1[0]) + DC * (x2[1] - x1[1]);
}

// A: mass 1. State (x1, v1); input the force of the coupling on it; outputs (x1, v1)
std::unique_ptr<OdeSubsystem> mass_1()
{
    return std::make_unique<OdeSubsystem>(
        vec{1.0, 0.0}, 1, 2,
        [](real_t, const vec &x, const vec &u) {
            return vec{x[1], (-K1 * x[0] + u[0]) / M1};
        },
        [](const vec &x, const vec &) {
            return vec{x[0], x[1]};
        },
        SUBSTEPS);
}

// B: mass 2 and the coupling spring. State (x2, v2); inputs (x1, v1); output the force on mass 1,
// which depends on the inputs directly
std::unique_ptr<OdeSubsystem> mass_2()
{
    return std::make_unique<OdeSubsystem>(
        vec{0.0, 0.0}, 2, 1,
        [](real_t, const vec &x, const vec &u) {
            return vec{x[1], (-K2 * x[0] - D2 * x[1] - coupling_force(x, u)) / M2};
        },
        [](const vec &x, const vec &u) { return vec{coupling_force(x, u)}; }, SUBSTEPS);
}

// the whole system, with one solver: (x1, v1, x2, v2)
OdeSubsystem whole_system(uint_t substeps)
{
    return OdeSubsystem(
        {1.0, 0.0, 0.0, 0.0}, 0, 1,
        [](real_t, const vec &x, const vec &)
        {
            const real_t f = coupling_force({x[2], x[3]}, {x[0], x[1]});
            return vec{x[1], (-K1 * x[0] + f) / M1, x[3], (-K2 * x[2] - D2 * x[3] - f) / M2};
        },
        [](const vec &x, const vec &) { return vec{x[0]}; }, substeps);
}

struct Scheme
{
    std::string name;
    CouplingConfig config;
};

std::vector<Scheme> schemes()
{
    CouplingConfig loose;
    CouplingConfig extrapolated;
    extrapolated.extrapolate = true;
    CouplingConfig seidel;
    seidel.scheme = CouplingScheme::GAUSS_SEIDEL;
    CouplingConfig strong;
    strong.max_iterations = 50;
    strong.tolerance = 1e-12;
    return {{"loose, inputs held", loose},
            {"loose, extrapolated", extrapolated},
            {"Gauss-Seidel", seidel},
            {"strong", strong}};
}

struct Run
{
    std::vector<real_t> x1; // at every macro step, the start included
    real_t max_abs_x1{0.0};
    real_t tries_per_step{0.0};
};

Run cosimulate(const CouplingConfig &config, real_t H, real_t T,
               bitsim::utils::concurrency::ThreadPool *pool = nullptr)
{
    auto a = mass_1();
    auto b = mass_2();
    CoSimulation cs(config);
    cs.add("mass 1", *a);
    cs.add("mass 2", *b);
    cs.connect("mass 1", 0, "mass 2", 0); // x1
    cs.connect("mass 1", 1, "mass 2", 1); // v1
    cs.connect("mass 2", 0, "mass 1", 0); // the force on mass 1
    cs.set_pool(pool);
    cs.initialise();
    Run run;
    run.x1.push_back(a->state()[0]);
    const auto n = static_cast<uint_t>(std::llround(T / H));
    for (uint_t s = 0; s < n; ++s)
    {
        cs.step(H);
        run.x1.push_back(a->state()[0]);
        run.max_abs_x1 = std::max(run.max_abs_x1, std::abs(a->state()[0]));
    }
    run.tries_per_step = static_cast<real_t>(cs.total_iterations()) / n;
    return run;
}

std::vector<real_t> reference(real_t H, real_t T)
{
    // Runge-Kutta steps of 10 microseconds
    auto whole = whole_system(static_cast<uint_t>(std::llround(H / 1e-5)));
    std::vector<real_t> x1{whole.state()[0]};
    const auto n = static_cast<uint_t>(std::llround(T / H));
    for (uint_t s = 0; s < n; ++s)
    {
        whole.do_step(s * H, H, {}, {});
        x1.push_back(whole.state()[0]);
    }
    return x1;
}

} // namespace

int main(int argc, char **argv)
{
    std::string csv_path = "oscillator.csv";
    for (int i = 1; i + 1 < argc; i += 2)
    {
        if (std::string(argv[i]) == "--csv")
        {
            csv_path = argv[i + 1];
        }
    }
    const real_t T = 2.0;
    const auto all = schemes();

    std::printf("1. The error of x1 at t = %.0f s, against the whole system with one solver\n\n",
                T);
    std::printf("   macro step [s]");
    for (const auto &s : all)
    {
        std::printf("   %20s", s.name.c_str());
    }
    std::printf("   tries per strong step\n");
    for (const real_t H : {0.04, 0.02, 0.01, 0.005, 0.0025})
    {
        const real_t ref = reference(H, T).back();
        std::printf("   %14.4f", H);
        real_t tries = 0.0;
        for (const auto &s : all)
        {
            const auto run = cosimulate(s.config, H, T);
            std::printf("   %20.2e", std::abs(run.x1.back() - ref));
            tries = run.tries_per_step;
        }
        std::printf("   %21.1f\n", tries);
    }

    const real_t H_long = 0.05;
    std::printf("\n2. A macro step of %.2f s, for 5 s: the largest |x1|; it starts at 1\n", H_long);
    for (const auto &s : all)
    {
        const auto run = cosimulate(s.config, H_long, 5.0);
        std::printf("   %20s   %10.3g%s\n", s.name.c_str(), run.max_abs_x1,
                    run.max_abs_x1 > 2.0 ? "   unstable" : "");
    }

    std::printf("\n3. The two subsystems on two threads\n");
    bitsim::utils::concurrency::ThreadPool pool(2);
    const auto alone = cosimulate(all.back().config, 0.01, T);
    const auto threads = cosimulate(all.back().config, 0.01, T, &pool);
    std::printf("   strong coupling gives the same x1 at every step, to the last bit: %s\n",
                alone.x1 == threads.x1 ? "true" : "false");

    // x1 over time with a macro step of 0.02 s, for the plot
    const real_t H = 0.02;
    const auto ref = reference(H, T);
    std::vector<Run> runs;
    for (const auto &s : all)
    {
        runs.push_back(cosimulate(s.config, H, T));
    }
    std::ofstream csv(csv_path);
    csv << "t,whole system";
    for (const auto &s : all)
    {
        csv << ",\"" << s.name << "\"";
    }
    csv << "\n";
    for (std::size_t k = 0; k < ref.size(); ++k)
    {
        csv << k * H << "," << ref[k];
        for (const auto &r : runs)
        {
            csv << "," << r.x1[k];
        }
        csv << "\n";
    }
    std::printf("\nx1 over time with a macro step of %.2f s is in %s\n", H, csv_path.c_str());
    return 0;
}

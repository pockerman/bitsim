#include "bitsim/cosim/co_simulation.h"
#include "bitsim/cosim/cosim_participant.h"
#include "bitsim/cosim/cosim_subsystem.h"
#include "bitsim/distributed/transport.h"
#include "bitsim/utils/concurrency/thread_pool.h"

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <thread>
#include <vector>

namespace
{
using bitsim::real_t;
using bitsim::uint_t;
using namespace bitsim::cosim;

// two masses on walls, joined by a stiff spring: mass 1 in subsystem A, mass 2 and the
// coupling spring in subsystem B
struct Oscillator
{
    real_t m1{1.0}, k1{10.0}, d1{0.0};
    real_t m2{1.0}, k2{1000.0}, d2{0.1};
    real_t kc{100.0}, dc{0.1};
    real_t x1_0{1.0};

    // A: state (x1, v1), input the force on mass 1, outputs (x1, v1)
    std::unique_ptr<OdeSubsystem> a(uint_t substeps) const
    {
        auto self = *this;
        return std::make_unique<OdeSubsystem>(
            std::vector<real_t>{x1_0, 0.0}, 1, 2,
            [self](real_t, const std::vector<real_t> &x, const std::vector<real_t> &u) {
                return std::vector<real_t>{x[1],
                                           (-self.k1 * x[0] - self.d1 * x[1] + u[0]) / self.m1};
            },
            [](const std::vector<real_t> &x, const std::vector<real_t> &) {
                return std::vector<real_t>{x[0], x[1]};
            },
            substeps);
    }

    real_t coupling_force_on_1(const std::vector<real_t> &x2, const std::vector<real_t> &u) const
    {
        return kc * (x2[0] - u[0]) + dc * (x2[1] - u[1]);
    }

    // B: state (x2, v2), inputs (x1, v1), output the force on mass 1
    std::unique_ptr<OdeSubsystem> b(uint_t substeps) const
    {
        auto self = *this;
        return std::make_unique<OdeSubsystem>(
            std::vector<real_t>{0.0, 0.0}, 2, 1,
            [self](real_t, const std::vector<real_t> &x, const std::vector<real_t> &u)
            {
                const real_t f = -self.coupling_force_on_1(x, u);
                return std::vector<real_t>{x[1], (-self.k2 * x[0] - self.d2 * x[1] + f) / self.m2};
            },
            [self](const std::vector<real_t> &x, const std::vector<real_t> &u)
            { return std::vector<real_t>{self.coupling_force_on_1(x, u)}; },
            substeps);
    }

    // the whole system, with one solver and small steps: x1 at time T
    real_t reference_x1(real_t T) const
    {
        auto self = *this;
        OdeSubsystem whole(
            {x1_0, 0.0, 0.0, 0.0}, 0, 1,
            [self](real_t, const std::vector<real_t> &x, const std::vector<real_t> &)
            {
                const real_t f1 = self.kc * (x[2] - x[0]) + self.dc * (x[3] - x[1]);
                return std::vector<real_t>{x[1], (-self.k1 * x[0] - self.d1 * x[1] + f1) / self.m1,
                                           x[3], (-self.k2 * x[2] - self.d2 * x[3] - f1) / self.m2};
            },
            [](const std::vector<real_t> &x, const std::vector<real_t> &)
            { return std::vector<real_t>{x[0]}; },
            200000);
        whole.do_step(0.0, T, {}, {});
        return whole.state()[0];
    }
};

void wire(CoSimulation &cs, CoSimSubsystem &a, CoSimSubsystem &b)
{
    cs.add("A", a);
    cs.add("B", b);
    cs.connect("A", 0, "B", 0);
    cs.connect("A", 1, "B", 1);
    cs.connect("B", 0, "A", 0);
}

CouplingLayout layout()
{
    CouplingLayout l;
    l.add("A", 1, 2);
    l.add("B", 2, 1);
    l.connect("A", 0, "B", 0);
    l.connect("A", 1, "B", 1);
    l.connect("B", 0, "A", 0);
    return l;
}

struct Run
{
    real_t x1{0.0};
    real_t max_abs_x1{0.0};
    uint_t iterations{0};
};

Run cosimulate(const CouplingConfig &config, real_t H, real_t T,
               bitsim::utils::concurrency::ThreadPool *pool = nullptr)
{
    Oscillator o;
    auto a = o.a(10);
    auto b = o.b(10);
    CoSimulation cs(config);
    wire(cs, *a, *b);
    cs.set_pool(pool);
    cs.initialise();
    Run run;
    const auto n = static_cast<uint_t>(std::llround(T / H));
    for (uint_t s = 0; s < n; ++s)
    {
        cs.step(H);
        run.max_abs_x1 = std::max(run.max_abs_x1, std::abs(a->state()[0]));
    }
    run.x1 = a->state()[0];
    run.iterations = cs.total_iterations();
    return run;
}

CouplingConfig strong()
{
    CouplingConfig c;
    c.max_iterations = 50;
    c.tolerance = 1e-12;
    return c;
}

TEST(TestOdeSubsystem, DecaysAndGoesBack)
{
    OdeSubsystem s(
        {1.0}, 1, 1,
        [](real_t, const std::vector<real_t> &x, const std::vector<real_t> &u)
        { return std::vector<real_t>{-x[0] + u[0]}; },
        [](const std::vector<real_t> &x, const std::vector<real_t> &) { return x; }, 100);
    s.save();
    s.do_step(0.0, 1.0, {0.0}, {0.0});
    EXPECT_NEAR(s.state()[0], std::exp(-1.0), 1e-9);
    s.restore();
    EXPECT_EQ(s.state()[0], 1.0);
    // an input rising from 0 to 1 over the step: x' = -x + t, x(0) = 1 -> x(1) = 2 / e
    s.do_step(0.0, 1.0, {0.0}, {1.0});
    EXPECT_NEAR(s.state()[0], 2.0 / std::exp(1.0), 1e-9);
    EXPECT_THROW(s.do_step(0.0, 1.0, {}, {0.0}), std::invalid_argument);
}

TEST(TestCouplingLayout, EveryInputOnce)
{
    CouplingLayout l;
    l.add("A", 1, 1);
    l.add("B", 1, 1);
    EXPECT_THROW(l.add("A", 1, 1), std::invalid_argument);
    EXPECT_THROW(l.connect("A", 1, "B", 0), std::invalid_argument);
    l.connect("A", 0, "B", 0);
    EXPECT_THROW(l.validate(), std::logic_error); // A's input is fed by nobody
    l.connect("B", 0, "A", 0);
    EXPECT_NO_THROW(l.validate());
    l.connect("B", 0, "A", 0);
    EXPECT_THROW(l.validate(), std::logic_error); // now twice
}

TEST(TestCoSimulation, StartsWithConsistentInputs)
{
    Oscillator o;
    o.x1_0 = 0.3;
    auto a = o.a(1);
    auto b = o.b(1);
    CoSimulation cs;
    wire(cs, *a, *b);
    cs.initialise();
    EXPECT_DOUBLE_EQ(cs.inputs("B")[0], 0.3);
    // B's output depends on its input: resolved, the force is kc (0 - 0.3)
    EXPECT_DOUBLE_EQ(cs.inputs("A")[0], -100.0 * 0.3);
}

TEST(TestCoSimulation, StrongCouplingApproachesTheWholeSystem)
{
    const real_t T = 2.0;
    const real_t ref = Oscillator().reference_x1(T);
    const real_t H = 0.01;
    const auto loose = cosimulate(CouplingConfig(), H, T);
    CouplingConfig gs;
    gs.scheme = CouplingScheme::GAUSS_SEIDEL;
    const auto seidel = cosimulate(gs, H, T);
    const auto tight = cosimulate(strong(), H, T);
    const real_t e_loose = std::abs(loose.x1 - ref);
    const real_t e_seidel = std::abs(seidel.x1 - ref);
    const real_t e_tight = std::abs(tight.x1 - ref);
    EXPECT_LT(e_seidel, e_loose);
    EXPECT_LT(e_tight, 0.1 * e_loose);
    EXPECT_LT(e_tight, 0.01); // second order: 0.0056 at H = 0.01
    EXPECT_GT(tight.iterations, 2 * static_cast<uint_t>(T / H));

    // halving the macro step makes the tightly coupled error about 4 times smaller
    const auto finer = cosimulate(strong(), H / 2, T);
    EXPECT_LT(std::abs(finer.x1 - ref), 0.4 * e_tight);
}

TEST(TestCoSimulation, LooseCouplingOfAStiffSpringBlowsUp)
{
    const real_t H = 0.05; // longer than the period of the stiff mode allows
    const auto loose = cosimulate(CouplingConfig(), H, 5.0);
    const auto tight = cosimulate(strong(), H, 5.0);
    EXPECT_GT(loose.max_abs_x1, 100.0);
    EXPECT_LT(tight.max_abs_x1, 1.5);
}

TEST(TestCoSimulation, TheSameOnThreads)
{
    bitsim::utils::concurrency::ThreadPool pool(2);
    const auto alone = cosimulate(strong(), 0.01, 1.0);
    const auto threaded = cosimulate(strong(), 0.01, 1.0, &pool);
    EXPECT_EQ(alone.x1, threaded.x1);
    EXPECT_EQ(alone.iterations, threaded.iterations);
}

TEST(TestCoSimulation, RejectsMisuse)
{
    Oscillator o;
    auto a = o.a(1);
    CoSimulation cs;
    cs.add("A", *a);
    EXPECT_THROW(cs.step(0.1), std::logic_error);
    EXPECT_THROW(cs.initialise(), std::logic_error); // A's input is not connected
    CouplingConfig gs;
    gs.scheme = CouplingScheme::GAUSS_SEIDEL;
    gs.max_iterations = 3;
    auto b = o.b(1);
    auto a2 = o.a(1);
    CoSimulation bad(gs);
    wire(bad, *a2, *b);
    EXPECT_THROW(bad.initialise(), std::invalid_argument);
}

// every subsystem a participant of its own, threads on an in-process bus
void distributed_matches(const CouplingConfig &config)
{
    const real_t H = 0.01;
    const uint_t N = 100;
    const auto one = cosimulate(config, H, H * N);

    Oscillator o;
    auto a = o.a(10);
    auto b = o.b(10);
    bitsim::distributed::InProcessBus bus;
    auto ta = bus.connect();
    auto tb = bus.connect();
    uint_t iterations_a = 0, iterations_b = 0;
    auto run = [&](const std::string &name, CoSimSubsystem &s, bitsim::distributed::Transport &t,
                   uint_t &iterations)
    {
        CoSimParticipant p(layout(), name, s, t, config);
        p.initialise();
        for (uint_t k = 0; k < N; ++k)
        {
            p.step(H);
        }
        iterations = p.total_iterations();
        p.leave();
    };
    std::thread pa([&] { run("A", *a, *ta, iterations_a); });
    std::thread pb([&] { run("B", *b, *tb, iterations_b); });
    pa.join();
    pb.join();
    EXPECT_EQ(a->state()[0], one.x1);
    EXPECT_EQ(iterations_a, one.iterations);
    EXPECT_EQ(iterations_b, one.iterations);
}

TEST(TestCoSimParticipant, MatchesOneProcessLoose) { distributed_matches(CouplingConfig()); }

TEST(TestCoSimParticipant, MatchesOneProcessStrong) { distributed_matches(strong()); }

TEST(TestCoSimParticipant, RejectsGaussSeidelAndWrongSizes)
{
    Oscillator o;
    auto a = o.a(1);
    bitsim::distributed::InProcessBus bus;
    auto t = bus.connect();
    CouplingConfig gs;
    gs.scheme = CouplingScheme::GAUSS_SEIDEL;
    EXPECT_THROW(CoSimParticipant(layout(), "A", *a, *t, gs), std::invalid_argument);
    EXPECT_THROW(CoSimParticipant(layout(), "B", *a, *t), std::invalid_argument);
}

} // namespace

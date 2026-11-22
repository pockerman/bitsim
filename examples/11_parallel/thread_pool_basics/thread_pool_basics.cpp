// The concurrency utilities of bitsim, one at a time: a pool of threads and the futures of its
// tasks, parallel_for over a range of indices, parallel_map with results in order, seeds for
// tasks that draw random numbers, and what happens when a task throws.
//
// Usage: thread_pool_basics

#include "bitsim/bitsim_types.h"
#include "bitsim/utils/concurrency/parallel.h"
#include "bitsim/utils/concurrency/thread_pool.h"
#include "bitsim/utils/io/print_utils.h"

#include <chrono>
#include <cmath>
#include <future>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <random>
#include <stdexcept>
#include <thread>
#include <vector>

namespace thread_pool_basics
{

using bitsim::real_t;
using bitsim::uint_t;
using namespace bitsim::utils::concurrency;

// A random walk of n steps of +1 or -1, from its own generator
real_t random_walk(uint_t seed, uint_t n)
{
    std::mt19937 rng(static_cast<std::mt19937::result_type>(seed));
    std::bernoulli_distribution up(0.5);
    real_t x = 0.0;
    for (uint_t i = 0; i < n; ++i)
    {
        x += up(rng) ? 1.0 : -1.0;
    }
    return x;
}

void run()
{
    bitsim::utils::io::print_title("1. A pool of threads");
    ThreadPool pool(4);
    std::cout << "  The pool has " << pool.size() << " threads; this machine has "
              << ThreadPool::default_n_threads() << " for the default pool\n";

    // submit() queues a task and gives back the future of its result
    auto area = pool.submit([](real_t r) { return M_PI * r * r; }, 2.0);
    auto inside = pool.submit([&pool] { return pool.is_worker_thread(); });
    std::cout << "  area of a circle of radius 2: " << area.get() << "\n";
    std::cout << "  the task ran on a thread of the pool: " << std::boolalpha << inside.get()
              << "; this thread is one of them: " << pool.is_worker_thread() << "\n";

    // an exception in a task comes out of the future's get()
    auto failing = pool.submit([]() -> real_t { throw std::domain_error("no square root of -1"); });
    try
    {
        failing.get();
    }
    catch (const std::domain_error &e)
    {
        std::cout << "  the failing task threw: " << e.what() << "\n\n";
    }

    bitsim::utils::io::print_title("2. parallel_for");
    // every index writes only its own element, so the threads never touch the same memory
    const uint_t N = 1000000;
    std::vector<real_t> roots(N);
    std::mutex mutex;
    std::map<std::thread::id, std::pair<uint_t, uint_t>> chunks; // thread -> first, last index
    parallel_for<uint_t>(
        0, N,
        [&](uint_t i)
        {
            roots[i] = std::sqrt(static_cast<real_t>(i));
            if (i % 10000 == 0)
            {
                // a lock only to record who did what; the work itself needs none
                std::lock_guard<std::mutex> lock(mutex);
                auto [itr, fresh] = chunks.try_emplace(std::this_thread::get_id(), i, i);
                if (!fresh)
                {
                    itr->second.first = std::min(itr->second.first, i);
                    itr->second.second = std::max(itr->second.second, i);
                }
            }
        },
        pool);
    std::cout << "  sqrt(999999) = " << roots.back() << ". The range was cut into "
              << pool.size() + 1 << " chunks of consecutive indices, one for the calling thread "
              << "and one for each thread of the pool; " << chunks.size()
              << " different threads ran them, since a thread that finishes early takes the next "
              << "chunk in the queue\n\n";

    bitsim::utils::io::print_title("3. parallel_map keeps the order");
    // the tasks finish in a scrambled order, the results still come back by index
    const auto squares = parallel_map(
        8,
        [](std::size_t i)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds((8 - i) * 5));
            return i * i;
        },
        pool);
    std::cout << "  squares:";
    for (const auto s : squares)
    {
        std::cout << " " << s;
    }
    std::cout << "\n\n";

    bitsim::utils::io::print_title("4. Seeds for random tasks");
    // each task gets its own generator, seeded with task_seed(seed, task); the results depend on
    // the seed and the task only, not on the threads that ran them
    const auto walk = [](std::size_t task) { return random_walk(task_seed(2024, task), 10000); };
    std::vector<real_t> on_one;
    for (std::size_t task = 0; task < 16; ++task)
    {
        on_one.push_back(walk(task));
    }
    const auto on_many = parallel_map(16, walk, default_thread_pool());
    std::cout << "  the end of 16 random walks of 10000 steps:";
    for (const auto x : on_many)
    {
        std::cout << " " << x;
    }
    std::cout << "\n  the same in a plain loop and on the threads: " << (on_one == on_many) << "\n";
    std::cout << "  task_seed(2024, 0) = " << task_seed(2024, 0)
              << ", task_seed(2024, 1) = " << task_seed(2024, 1) << "\n\n";

    bitsim::utils::io::print_title("5. Exceptions and nesting");
    try
    {
        parallel_for(
            0, 100,
            [](int i)
            {
                if (i == 42)
                {
                    throw std::out_of_range("index 42 is not allowed");
                }
            },
            pool);
    }
    catch (const std::out_of_range &e)
    {
        std::cout << "  parallel_for waited for every chunk, then rethrew: " << e.what() << "\n";
    }

    // a parallel_for inside a task of the same pool runs on that task's thread, so a pool of one
    // thread cannot wait for itself
    ThreadPool one_thread(1);
    auto nested = one_thread.submit(
        [&one_thread]
        {
            std::vector<int> v(100, 1);
            parallel_for<uint_t>(
                0, v.size(), [&v](uint_t i) { v[i] *= 3; }, one_thread);
            int sum = 0;
            for (const int x : v)
            {
                sum += x;
            }
            return sum;
        });
    std::cout << "  a parallel_for nested in a task of a one-thread pool finished, sum = "
              << nested.get() << "\n";
}

} // namespace thread_pool_basics

int main()
{
    thread_pool_basics::run();
    return 0;
}

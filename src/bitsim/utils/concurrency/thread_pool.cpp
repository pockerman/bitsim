#include "bitsim/utils/concurrency/thread_pool.h"

#include <cstdlib>
#include <string>

namespace bitsim
{
namespace utils::concurrency
{

namespace
{
// The pool whose task the current thread runs, if any
thread_local const ThreadPool *current_pool = nullptr;
} // namespace

uint_t ThreadPool::default_n_threads()
{
    if (const char *value = std::getenv("BITSIM_NUM_THREADS"))
    {
        try
        {
            const long n = std::stol(value);
            if (n > 0)
            {
                return static_cast<uint_t>(n);
            }
        }
        catch (const std::exception &)
        {
            // not a number: fall back to the number of cores
        }
    }
    const auto n_cores = std::thread::hardware_concurrency();
    return n_cores > 0 ? static_cast<uint_t>(n_cores) : 1;
}

ThreadPool::ThreadPool(uint_t n_threads)
{
    if (n_threads == 0)
    {
        n_threads = default_n_threads();
    }
    workers_.reserve(n_threads);
    for (uint_t t = 0; t < n_threads; ++t)
    {
        workers_.emplace_back(&ThreadPool::work_, this);
    }
}

ThreadPool::~ThreadPool()
{
    // the threads still take what is queued, and stop once the queue is empty
    tasks_.close();
    for (auto &worker : workers_)
    {
        if (worker.joinable())
        {
            worker.join();
        }
    }
}

bool ThreadPool::is_worker_thread() const noexcept { return current_pool == this; }

void ThreadPool::work_()
{
    current_pool = this;
    while (auto task = tasks_.pop())
    {
        // a packaged_task keeps the exception of the task for its future
        (*task)();
    }
    current_pool = nullptr;
}

ThreadPool &default_thread_pool()
{
    static ThreadPool pool;
    return pool;
}

} // namespace utils::concurrency
} // namespace bitsim

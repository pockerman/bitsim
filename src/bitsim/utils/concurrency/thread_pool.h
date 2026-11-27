#ifndef BITSIM_UTILS_CONCURRENCY_THREAD_POOL_H
#define BITSIM_UTILS_CONCURRENCY_THREAD_POOL_H

#include "bitsim/bitsim_types.h"
#include "bitsim/utils/concurrency/concurrent_queue.h"

#include <boost/noncopyable.hpp>

#include <functional>
#include <future>
#include <memory>
#include <stdexcept>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

namespace bitsim
{
namespace utils::concurrency
{

///
/// \brief A fixed number of threads that run the tasks given to them, in the order given.
///
/// submit() returns a std::future for the task's result; an exception the task throws comes
/// out of the future's get(). The destructor runs the tasks still queued, then joins the threads.
///
/// Most code needs only parallel_for() and parallel_map() in
/// bitsim/utils/concurrency/parallel.h, which use default_thread_pool().
///
class ThreadPool : private boost::noncopyable
{
  public:
    ///
    /// \brief The number of threads to use when none is given: the environment variable
    /// BITSIM_NUM_THREADS if it is a positive number, otherwise the number of cores
    ///
    static uint_t default_n_threads();

    ///
    /// \brief Start n_threads threads; 0 means default_n_threads()
    ///
    explicit ThreadPool(uint_t n_threads = 0);

    ///
    /// \brief Run the tasks still queued, then join the threads
    ///
    ~ThreadPool();

    ///
    /// \brief The number of threads
    ///
    uint_t size() const noexcept { return static_cast<uint_t>(workers_.size()); }

    ///
    /// \brief Whether the calling thread is one of this pool's threads.
    ///
    /// A task that waits for other tasks of its own pool can leave no thread to run them;
    /// parallel_for() checks this and runs serially inside a task.
    ///
    bool is_worker_thread() const noexcept;

    ///
    /// \brief Queue f(args...) to run on one of the threads
    /// \return the future of its result
    ///
    template <typename F, typename... Args>
    auto submit(F &&f, Args &&...args) -> std::future<std::invoke_result_t<F, Args...>>;

  private:
    void work_();

    ConcurrentQueue<std::function<void()>> tasks_;
    std::vector<std::thread> workers_;
};

///
/// \brief The pool that parallel_for() and parallel_map() use when given none. It starts
/// ThreadPool::default_n_threads() threads the first time it is used.
///
ThreadPool &default_thread_pool();

template <typename F, typename... Args>
auto ThreadPool::submit(F &&f, Args &&...args) -> std::future<std::invoke_result_t<F, Args...>>
{
    using result_type = std::invoke_result_t<F, Args...>;

    // std::function needs a copyable callable, and a packaged_task is not
    auto task = std::make_shared<std::packaged_task<result_type()>>(
        [f = std::forward<F>(f), ... args = std::forward<Args>(args)]() mutable -> result_type
        { return std::invoke(std::move(f), std::move(args)...); });

    auto result = task->get_future();
    if (!tasks_.push([task]() { (*task)(); }))
    {
        throw std::logic_error("ThreadPool: submit() on a pool that is shutting down");
    }
    return result;
}

} // namespace utils::concurrency
} // namespace bitsim

#endif // BITSIM_UTILS_CONCURRENCY_THREAD_POOL_H

#ifndef BITSIM_UTILS_CONCURRENCY_PARALLEL_H
#define BITSIM_UTILS_CONCURRENCY_PARALLEL_H

#include "bitsim/bitsim_types.h"
#include "bitsim/utils/concurrency/thread_pool.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <future>
#include <optional>
#include <type_traits>
#include <vector>

namespace bitsim
{
namespace utils::concurrency
{

///
/// \brief Call f(i) for every i in [begin, end), spread over the threads of pool.
///
/// The range is cut into at most pool.size() + 1 chunks of consecutive indices, and the calling
/// thread runs one of them, so a pool of one thread already uses two. f must be safe to call
/// from several threads at once: each call should write only to what belongs to its index.
/// The function returns when every call has returned; if calls throw, it rethrows the first
/// exception after all chunks are done. Called from a task of pool, it runs the calls in order on
/// the calling thread, since waiting for the pool from inside it could leave no thread to run them.
///
/// \param begin the first index
/// \param end one past the last index
/// \param f the work for one index
/// \param pool the threads to use
/// \param grain the fewest indices in a chunk; raise it when f(i) is very cheap
///
template <typename Index, typename F>
void parallel_for(Index begin, Index end, F &&f, ThreadPool &pool = default_thread_pool(),
                  Index grain = 1)
{
    static_assert(std::is_integral_v<Index>, "parallel_for needs an integral index");
    if (end <= begin)
    {
        return;
    }

    const auto n = static_cast<std::size_t>(end - begin);
    const auto min_chunk = static_cast<std::size_t>(std::max<Index>(grain, 1));
    const std::size_t max_chunks = static_cast<std::size_t>(pool.size()) + 1;
    const std::size_t n_chunks = std::min(max_chunks, (n + min_chunk - 1) / min_chunk);

    auto run_chunk = [&f, begin, n, n_chunks](std::size_t c)
    {
        // chunk c covers [n * c / n_chunks, n * (c + 1) / n_chunks)
        const auto first = begin + static_cast<Index>(n * c / n_chunks);
        const auto last = begin + static_cast<Index>(n * (c + 1) / n_chunks);
        for (Index i = first; i < last; ++i)
        {
            f(i);
        }
    };

    if (n_chunks <= 1 || pool.is_worker_thread())
    {
        for (std::size_t c = 0; c < n_chunks; ++c)
        {
            run_chunk(c);
        }
        return;
    }

    std::vector<std::future<void>> pending;
    pending.reserve(n_chunks - 1);
    for (std::size_t c = 1; c < n_chunks; ++c)
    {
        pending.push_back(pool.submit(run_chunk, c));
    }

    // the chunks refer to f, so wait for all of them before leaving, even if one throws
    std::exception_ptr error;
    try
    {
        run_chunk(0);
    }
    catch (...)
    {
        error = std::current_exception();
    }
    for (auto &chunk : pending)
    {
        try
        {
            chunk.get();
        }
        catch (...)
        {
            if (!error)
            {
                error = std::current_exception();
            }
        }
    }
    if (error)
    {
        std::rethrow_exception(error);
    }
}

///
/// \brief Compute f(0), f(1), ..., f(n - 1) on the threads of pool.
///
/// The results come back in the order of their index, whatever order the threads finish in, so
/// the output does not depend on the number of threads. A typical use is to run one simulation or
/// one training per seed or per parameter value. The rules of parallel_for() apply.
///
/// \return the results, results[i] = f(i)
///
template <typename F>
auto parallel_map(std::size_t n, F &&f, ThreadPool &pool = default_thread_pool())
    -> std::vector<std::invoke_result_t<F &, std::size_t>>
{
    typedef std::invoke_result_t<F &, std::size_t> result_type;
    static_assert(!std::is_void_v<result_type>, "parallel_map needs f to return a value; "
                                                "use parallel_for for work without a result");

    // optional, so that the result type need not be default constructible
    std::vector<std::optional<result_type>> slots(n);
    parallel_for<std::size_t>(
        0, n, [&slots, &f](std::size_t i) { slots[i].emplace(f(i)); }, pool);

    std::vector<result_type> results;
    results.reserve(n);
    for (auto &slot : slots)
    {
        results.push_back(std::move(*slot));
    }
    return results;
}

///
/// \brief A seed for task task, made from base_seed.
///
/// Tasks that draw random numbers need a generator each, since sharing one between threads is a
/// data race. Seeding them with base_seed + task gives generators whose first numbers are
/// alike; this mixes the two numbers with SplitMix64, so that every task gets an unrelated seed
/// that still depends only on base_seed and task.
///
inline uint_t task_seed(uint_t base_seed, uint_t task) noexcept
{
    std::uint64_t z = static_cast<std::uint64_t>(base_seed) +
                      0x9E3779B97F4A7C15ULL * (static_cast<std::uint64_t>(task) + 1);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return static_cast<uint_t>(z ^ (z >> 31));
}

} // namespace utils::concurrency
} // namespace bitsim

#endif // BITSIM_UTILS_CONCURRENCY_PARALLEL_H

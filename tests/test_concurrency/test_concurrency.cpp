#include "bitsim/utils/concurrency/concurrent_queue.h"
#include "bitsim/utils/concurrency/parallel.h"
#include "bitsim/utils/concurrency/thread_pool.h"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <numeric>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{
using bitsim::uint_t;
using namespace bitsim::utils::concurrency;
using namespace std::chrono_literals;

TEST(TestConcurrentQueue, KeepsTheOrderOfThePushes)
{
    ConcurrentQueue<int> queue;
    for (int i = 0; i < 5; ++i)
    {
        EXPECT_TRUE(queue.push(i));
    }
    EXPECT_EQ(queue.size(), 5u);
    for (int i = 0; i < 5; ++i)
    {
        EXPECT_EQ(queue.try_pop(), i);
    }
    EXPECT_TRUE(queue.empty());
    EXPECT_EQ(queue.try_pop(), std::nullopt);
}

TEST(TestConcurrentQueue, PopForGivesUpAfterTheTimeout)
{
    ConcurrentQueue<int> queue;
    const auto start = std::chrono::steady_clock::now();
    EXPECT_EQ(queue.pop_for(20ms), std::nullopt);
    EXPECT_GE(std::chrono::steady_clock::now() - start, 15ms);
}

TEST(TestConcurrentQueue, PopWaitsForAnotherThread)
{
    ConcurrentQueue<std::string> queue;
    std::thread producer(
        [&queue]
        {
            std::this_thread::sleep_for(10ms);
            queue.push("hello");
        });
    EXPECT_EQ(queue.pop(), "hello");
    producer.join();
}

TEST(TestConcurrentQueue, CloseWakesTheWaitersAndDropsNewItems)
{
    ConcurrentQueue<int> queue;
    queue.push(1);
    std::thread closer(
        [&queue]
        {
            std::this_thread::sleep_for(10ms);
            queue.close();
        });
    EXPECT_EQ(queue.pop(), 1);            // what was queued still comes out
    EXPECT_EQ(queue.pop(), std::nullopt); // then the waiter wakes empty-handed
    closer.join();
    EXPECT_TRUE(queue.is_closed());
    EXPECT_FALSE(queue.push(2));
    EXPECT_TRUE(queue.empty());
}

TEST(TestConcurrentQueue, HoldsMoveOnlyItems)
{
    ConcurrentQueue<std::unique_ptr<int>> queue;
    queue.push(std::make_unique<int>(7));
    auto item = queue.pop();
    ASSERT_TRUE(item.has_value());
    EXPECT_EQ(**item, 7);
}

TEST(TestConcurrentQueue, ManyProducersLoseNothing)
{
    ConcurrentQueue<int> queue;
    std::vector<std::thread> producers;
    for (int p = 0; p < 4; ++p)
    {
        producers.emplace_back(
            [&queue, p]
            {
                for (int i = 0; i < 1000; ++i)
                {
                    queue.push(p * 1000 + i);
                }
            });
    }
    for (auto &producer : producers)
    {
        producer.join();
    }
    std::set<int> seen;
    while (auto item = queue.try_pop())
    {
        seen.insert(*item);
    }
    EXPECT_EQ(seen.size(), 4000u);
}

TEST(TestThreadPool, StartsTheThreadsAskedFor)
{
    ThreadPool pool(3);
    EXPECT_EQ(pool.size(), 3u);
    EXPECT_FALSE(pool.is_worker_thread());
    EXPECT_GE(ThreadPool::default_n_threads(), 1u);
}

TEST(TestThreadPool, SubmitReturnsTheResult)
{
    ThreadPool pool(2);
    auto sum = pool.submit([](int a, int b) { return a + b; }, 2, 3);
    auto inside = pool.submit([&pool] { return pool.is_worker_thread(); });
    EXPECT_EQ(sum.get(), 5);
    EXPECT_TRUE(inside.get());
}

TEST(TestThreadPool, TheFutureRethrowsTheTasksException)
{
    ThreadPool pool(1);
    auto failing = pool.submit([]() -> int { throw std::runtime_error("boom"); });
    EXPECT_THROW(failing.get(), std::runtime_error);
    // the thread survives the exception
    EXPECT_EQ(pool.submit([] { return 1; }).get(), 1);
}

TEST(TestThreadPool, TheDestructorRunsTheQueuedTasks)
{
    std::atomic<int> done{0};
    {
        ThreadPool pool(1);
        for (int i = 0; i < 20; ++i)
        {
            pool.submit(
                [&done]
                {
                    std::this_thread::sleep_for(1ms);
                    ++done;
                });
        }
    }
    EXPECT_EQ(done.load(), 20);
}

TEST(TestParallelFor, VisitsEveryIndexOnce)
{
    ThreadPool pool(4);
    std::vector<int> visits(1003, 0);
    parallel_for<uint_t>(0, visits.size(), [&visits](uint_t i) { ++visits[i]; }, pool);
    for (const int v : visits)
    {
        EXPECT_EQ(v, 1);
    }
}

TEST(TestParallelFor, HandlesEmptyAndOffsetRanges)
{
    ThreadPool pool(2);
    int calls = 0;
    parallel_for(5, 5, [&calls](int) { ++calls; }, pool);
    parallel_for(5, 2, [&calls](int) { ++calls; }, pool);
    EXPECT_EQ(calls, 0);

    std::vector<int> seen(10, 0);
    parallel_for(-3, 7, [&seen](int i) { seen[i + 3] = i; }, pool);
    for (int i = -3; i < 7; ++i)
    {
        EXPECT_EQ(seen[i + 3], i);
    }
}

TEST(TestParallelFor, RethrowsAfterEveryChunkIsDone)
{
    ThreadPool pool(3);
    std::atomic<int> calls{0};
    EXPECT_THROW(parallel_for(
                     0, 100,
                     [&calls](int i)
                     {
                         ++calls;
                         if (i == 50)
                         {
                             throw std::invalid_argument("bad index");
                         }
                     },
                     pool),
                 std::invalid_argument);
    // the chunk that threw stops, the others run to their end
    EXPECT_GT(calls.load(), 50);
}

TEST(TestParallelFor, RunsSeriallyInsideATaskOfItsOwnPool)
{
    // with one thread, a nested parallel_for that waited for the pool would never finish
    ThreadPool pool(1);
    auto total = pool.submit(
        [&pool]
        {
            std::vector<int> v(100, 1);
            parallel_for<uint_t>(0, v.size(), [&v](uint_t i) { v[i] *= 2; }, pool);
            return std::accumulate(v.begin(), v.end(), 0);
        });
    ASSERT_EQ(total.wait_for(5s), std::future_status::ready);
    EXPECT_EQ(total.get(), 200);
}

TEST(TestParallelMap, KeepsTheOrderOfTheIndices)
{
    ThreadPool pool(4);
    const auto squares = parallel_map(
        50,
        [](std::size_t i)
        {
            // finish in a scrambled order
            std::this_thread::sleep_for(std::chrono::microseconds((i * 37) % 11));
            return i * i;
        },
        pool);
    ASSERT_EQ(squares.size(), 50u);
    for (std::size_t i = 0; i < squares.size(); ++i)
    {
        EXPECT_EQ(squares[i], i * i);
    }
}

TEST(TestParallelMap, AcceptsResultsWithoutADefaultConstructor)
{
    struct Labelled
    {
        explicit Labelled(std::size_t i) : label("run " + std::to_string(i)) {}
        std::string label;
    };
    const auto runs = parallel_map(3, [](std::size_t i) { return Labelled(i); });
    EXPECT_EQ(runs[2].label, "run 2");
}

TEST(TestTaskSeed, IsRepeatableAndDiffersPerTask)
{
    EXPECT_EQ(task_seed(42, 3), task_seed(42, 3));
    std::set<uint_t> seeds;
    for (uint_t t = 0; t < 100; ++t)
    {
        seeds.insert(task_seed(42, t));
    }
    EXPECT_EQ(seeds.size(), 100u);
    EXPECT_NE(task_seed(42, 0), task_seed(43, 0));
}

} // namespace

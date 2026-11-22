\page bitsim_example_thread_pool_basics Threads, parallel loops and seeds

Many computations in robotics are a lot of small pieces of work that do not depend on each other:
one simulation per seed, one picture per frame, one ray per beam of a sensor. bitsim's
`bitsim/utils/concurrency` runs such work on all the cores. This example shows its parts one at a
time.

## What you need

Nothing beyond bitsim itself. See \ref bitsim_install.

## The pieces

| | What it does |
| --- | --- |
| \ref bitsim::utils::concurrency::ThreadPool "ThreadPool" | a fixed number of threads; `submit(f, args...)` queues a task and returns the `std::future` of its result |
| `parallel_for(begin, end, f)` | calls `f(i)` for every index, cut into chunks of consecutive indices; returns when all are done |
| `parallel_map(n, f)` | returns `f(0), ..., f(n - 1)`, in the order of the index |
| `task_seed(seed, task)` | an unrelated seed for every task, for tasks that draw random numbers |
| `default_thread_pool()` | the pool the two functions use when given none: one thread per core, or `BITSIM_NUM_THREADS` |

The rule for the work: each call should write only to what is its own, an element of a vector
of results for example, and read what nobody writes. Then the threads never need a lock.

## Running the example

@code{.sh}
./thread_pool_basics
@endcode

## Output

@code
1. A pool of threads
  The pool has 4 threads; this machine has 8 for the default pool
  area of a circle of radius 2: 12.5664
  the task ran on a thread of the pool: true; this thread is one of them: false
  the failing task threw: no square root of -1

2. parallel_for
  sqrt(999999) = 999.999. The range was cut into 5 chunks of consecutive indices, one for the calling thread and one for each thread of the pool; 4 different threads ran them, since a thread that finishes early takes the next chunk in the queue

3. parallel_map keeps the order
  squares: 0 1 4 9 16 25 36 49

4. Seeds for random tasks
  the end of 16 random walks of 10000 steps: -102 24 -34 -48 -2 -38 -42 16 -228 -108 66 -18 -170 172 78 -82
  the same in a plain loop and on the threads: true
  task_seed(2024, 0) = 11487996472437173461, task_seed(2024, 1) = 1793612131670815442

5. Exceptions and nesting
  parallel_for waited for every chunk, then rethrew: index 42 is not allowed
  a parallel_for nested in a task of a one-thread pool finished, sum = 300
@endcode

The number of threads that ran the chunks in part 2 changes from run to run; everything else is
the same every time.

1. A task that throws does not take its thread down: the exception waits in the future, and
   `get()` throws it.
2. `parallel_for` gives the calling thread a chunk as well, so a pool of 4 threads works on 5
   chunks at once. A pool of one thread therefore already runs two pieces of work at a time; to
   time one thread, use a plain loop, as \ref bitsim_example_parallel_monte_carlo does.
3. The tasks of part 3 sleep longer the smaller their index, so they finish backwards, but
   `parallel_map` puts every result in the slot of its index.
4. A generator shared by threads is a data race, and even with a lock the numbers each task got
   would depend on which thread asked first. With a generator per task, seeded with
   `task_seed()`, the walks are the same in a plain loop and on 8 threads. Seeds `seed + task`
   would work too, but generators seeded with neighbouring numbers start with numbers that are
   alike; `task_seed` mixes the two with SplitMix64.
5. If calls throw, `parallel_for` still waits for every chunk, since they all use `f`, and then
   rethrows the first exception. A `parallel_for` called from a task of its own pool runs on that
   thread, in order: waiting for the pool from inside it could leave no thread to do the work.

## Things to try

1. Make `roots` a `std::vector<bool>` and set `roots[i] = i % 2`. Why is that a data race, when a
   `std::vector<char>` is not?
2. Time part 2 with a grain of 1 and of 100000 (the last argument of `parallel_for`).
3. Seed the walks with `2024 + task` and compare the first steps of neighbouring walks.

## The code

\include thread_pool_basics.cpp

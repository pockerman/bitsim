// A gyroscope on one thread, a heading estimator on another, and a ConcurrentQueue between them.
//
// The sensor thread produces a reading of the turn rate every 5 ms for 2 s, the way a driver or
// an MQTT client receives messages on a thread of its own. The estimator takes the readings from
// the queue and integrates them into a heading. A second, slow estimator shows the queue filling
// up when the consumer cannot keep up, and close() shows how the producer tells the consumer that
// nothing more is coming.
//
// Usage: producer_consumer

#include "bitsim/bitsim_types.h"
#include "bitsim/utils/concurrency/concurrent_queue.h"
#include "bitsim/utils/io/print_utils.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <random>
#include <thread>

namespace producer_consumer
{

using bitsim::real_t;
using bitsim::uint_t;
using bitsim::utils::concurrency::ConcurrentQueue;
using clock_type = std::chrono::steady_clock;
using namespace std::chrono_literals;

// One reading of the gyroscope
struct GyroReading
{
    uint_t index;
    real_t time; // [s]
    real_t rate; // [rad/s]
};

const real_t PERIOD = 0.005;   // a reading every 5 ms
const uint_t N_READINGS = 400; // for 2 s
const real_t TRUE_RATE = 0.5;  // the robot turns at 0.5 rad/s

// Produce N_READINGS readings at the sensor's rate, then close the queue
void sensor(ConcurrentQueue<GyroReading> &queue)
{
    std::mt19937 rng(7);
    std::normal_distribution<real_t> noise(0.0, 0.02);
    const auto start = clock_type::now();
    for (uint_t k = 0; k < N_READINGS; ++k)
    {
        std::this_thread::sleep_until(start + std::chrono::duration<real_t>(k * PERIOD));
        queue.push({k, k * PERIOD, TRUE_RATE + noise(rng)});
    }
    // nothing more is coming: a consumer waiting in pop() wakes up empty handed
    queue.close();
}

struct Estimate
{
    uint_t n_readings{0};
    real_t heading{0.0};
    std::size_t max_backlog{0};
    uint_t n_timeouts{0};
};

// Integrate the readings into a heading; work_per_reading makes the consumer slower
Estimate estimator(ConcurrentQueue<GyroReading> &queue, std::chrono::microseconds work_per_reading)
{
    Estimate e;
    while (true)
    {
        // wait at most 20 ms for a reading, so that a dead sensor would be noticed
        auto reading = queue.pop_for(20ms);
        if (!reading)
        {
            if (queue.is_closed() && queue.empty())
            {
                break; // the sensor is done
            }
            ++e.n_timeouts;
            continue;
        }
        e.heading += reading->rate * PERIOD;
        ++e.n_readings;
        e.max_backlog = std::max(e.max_backlog, queue.size());
        std::this_thread::sleep_for(work_per_reading);
    }
    return e;
}

void report(const std::string &name, const Estimate &e, real_t seconds)
{
    std::cout << "  " << name << ": " << e.n_readings << " readings in " << std::fixed
              << std::setprecision(2) << seconds << " s, heading " << std::setprecision(4)
              << e.heading << " rad (true " << TRUE_RATE * N_READINGS * PERIOD << "), at most "
              << e.max_backlog << " readings waiting, " << e.n_timeouts << " waits timed out\n";
}

void run_with(const std::string &name, std::chrono::microseconds work_per_reading)
{
    ConcurrentQueue<GyroReading> queue;
    const auto start = clock_type::now();
    std::thread producer(sensor, std::ref(queue));
    const Estimate e = estimator(queue, work_per_reading);
    producer.join();
    report(name, e, std::chrono::duration<real_t>(clock_type::now() - start).count());
}

void run()
{
    bitsim::utils::io::print_title("1. A consumer that keeps up");
    // 0.1 ms of work per reading, well within the 5 ms between readings
    run_with("fast estimator", 100us);
    std::cout << "\n";

    bitsim::utils::io::print_title("2. A consumer that falls behind");
    // 8 ms of work per reading: the readings pile up in the queue, none is lost, and the
    // estimate is late but the same
    run_with("slow estimator", 8ms);
    std::cout << "\n";

    bitsim::utils::io::print_title("3. Draining what is there");
    ConcurrentQueue<GyroReading> queue;
    for (uint_t k = 0; k < 5; ++k)
    {
        queue.push({k, k * PERIOD, TRUE_RATE});
    }
    uint_t drained = 0;
    // try_pop never waits: take everything that has arrived, then go on with the control loop
    while (queue.try_pop())
    {
        ++drained;
    }
    queue.close();
    std::cout << "  try_pop took " << drained << " readings without waiting; after close(), push() "
              << "gives " << std::boolalpha << queue.push({5, 0.0, 0.0}) << "\n";
}

} // namespace producer_consumer

int main()
{
    producer_consumer::run();
    return 0;
}

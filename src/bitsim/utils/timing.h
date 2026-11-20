#ifndef BITSIM_UTILS_TIMING_H
#define BITSIM_UTILS_TIMING_H

/**
 * \file timing.h
 * \brief Wall clock helpers for running a simulation in real time.
 */

#include "bitsim/bitsim_types.h"

#include <chrono>
#include <thread>

namespace bitsim
{
namespace utils
{

using WallClock = std::chrono::steady_clock;

///
/// @brief The seconds from one time point to another
///
inline real_t seconds_between(WallClock::time_point from, WallClock::time_point to)
{
    return std::chrono::duration<real_t>(to - from).count();
}

///
/// @brief The milliseconds since a time point
///
inline real_t milliseconds_since(WallClock::time_point t)
{
    return std::chrono::duration<real_t, std::milli>(WallClock::now() - t).count();
}

///
/// @brief Sleep until step steps of step_size [s] after start, so that a loop that calls this
/// every step keeps to real time without its errors adding up
///
inline void sleep_until_step(WallClock::time_point start, long step, real_t step_size)
{
    std::this_thread::sleep_until(
        start + std::chrono::microseconds(static_cast<long>(1.0e6 * step_size * step)));
}

} // namespace utils
} // namespace bitsim

#endif // BITSIM_UTILS_TIMING_H

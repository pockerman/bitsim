#ifndef BITSIM_TRAJECTORY_TIME_SCALING_H
#define BITSIM_TRAJECTORY_TIME_SCALING_H

#include "bitsim/bitsim_types.h"

#include <memory>
#include <vector>

namespace bitsim
{
namespace trajectory
{

/**
 * @class TimeScaling
 * @brief How fast to go along a path: the share s(t) of the path done at time t, from s(0) = 0
 * to s(T) = 1, with its rates \f$\dot s\f$ and \f$\ddot s\f$.
 *
 * A point to point motion from \f$q_0\f$ to \f$q_1\f$ is then
 * \f$q(t) = q_0 + s(t)(q_1 - q_0)\f$, with velocity \f$\dot s(t)(q_1 - q_0)\f$ and acceleration
 * \f$\ddot s(t)(q_1 - q_0)\f$: the path is fixed, and the time scaling says how it is timed.
 */
class TimeScaling
{
  public:
    virtual ~TimeScaling() = default;

    ///
    /// @brief The duration T [s]
    ///
    real_t duration() const noexcept { return T_; }

    ///
    /// @brief s(t); 0 before the start and 1 after the end
    ///
    virtual real_t s(real_t t) const = 0;

    ///
    /// @brief \f$\dot s(t)\f$ [1/s]
    ///
    virtual real_t ds(real_t t) const = 0;

    ///
    /// @brief \f$\ddot s(t)\f$ [1/s^2]
    ///
    virtual real_t dds(real_t t) const = 0;

  protected:
    explicit TimeScaling(real_t duration);

    // t clamped to [0, T]
    real_t clamp_(real_t t) const noexcept;

    real_t T_;
};

/**
 * @class CubicTimeScaling
 * @brief The cubic polynomial that starts and ends at rest,
 * \f$s(t) = 3\tau^2 - 2\tau^3\f$ with \f$\tau = t / T\f$.
 *
 * The velocity is zero at both ends, but the acceleration jumps from 0 to \f$6/T^2\f$ at the
 * start and back at the end: an infinite jerk, which shakes a real arm.
 */
class CubicTimeScaling : public TimeScaling
{
  public:
    explicit CubicTimeScaling(real_t duration) : TimeScaling(duration) {}

    real_t s(real_t t) const override;
    real_t ds(real_t t) const override;
    real_t dds(real_t t) const override;
};

/**
 * @class QuinticTimeScaling
 * @brief The quintic polynomial that starts and ends at rest with no acceleration,
 * \f$s(t) = 10\tau^3 - 15\tau^4 + 6\tau^5\f$.
 *
 * Velocity and acceleration are zero at both ends, so the motion starts and stops smoothly. The
 * price is a higher peak velocity, \f$1.875/T\f$ against the cubic's \f$1.5/T\f$.
 */
class QuinticTimeScaling : public TimeScaling
{
  public:
    explicit QuinticTimeScaling(real_t duration) : TimeScaling(duration) {}

    real_t s(real_t t) const override;
    real_t ds(real_t t) const override;
    real_t dds(real_t t) const override;
};

/**
 * @class TrapezoidalTimeScaling
 * @brief Accelerate at a constant rate, cruise at constant speed, and brake at the same rate:
 * the speed over time is a trapezoid.
 *
 * It is what motor controllers do when they are given a top speed and an acceleration. For a
 * motion of duration T with a cruise speed of \f$\dot s = v\f$ the acceleration lasts
 * \f$t_a = T - 1/v\f$, which needs \f$1/T < v \le 2/T\f$; at \f$v = 2/T\f$ there is no cruise,
 * and the trapezoid becomes a triangle.
 */
class TrapezoidalTimeScaling : public TimeScaling
{
  public:
    ///
    /// @param duration T [s]
    /// @param cruise_speed \f$v\f$, the speed of s while cruising [1/s], in \f$(1/T, 2/T]\f$
    ///
    TrapezoidalTimeScaling(real_t duration, real_t cruise_speed);

    ///
    /// @brief The trapezoid that takes duration and spends a share of it, accel_share, in each
    /// of the speeding up and the slowing down. accel_share is in (0, 1/2]
    ///
    static TrapezoidalTimeScaling with_accel_share(real_t duration, real_t accel_share);

    real_t s(real_t t) const override;
    real_t ds(real_t t) const override;
    real_t dds(real_t t) const override;

    real_t cruise_speed() const noexcept { return v_; }
    real_t acceleration() const noexcept { return a_; }
    real_t accel_time() const noexcept { return ta_; }

  private:
    real_t v_;
    real_t a_;
    real_t ta_;
};

/**
 * @class PointToPoint
 * @brief A straight motion from one point of any dimension, such as the joint angles of an arm,
 * to another, timed by a TimeScaling.
 */
class PointToPoint
{
  public:
    PointToPoint(std::vector<real_t> start, std::vector<real_t> end,
                 std::shared_ptr<const TimeScaling> timing);

    std::vector<real_t> position(real_t t) const;
    std::vector<real_t> velocity(real_t t) const;
    std::vector<real_t> acceleration(real_t t) const;

    real_t duration() const noexcept { return timing_->duration(); }
    const TimeScaling &timing() const noexcept { return *timing_; }

  private:
    std::vector<real_t> start_;
    std::vector<real_t> delta_;
    std::shared_ptr<const TimeScaling> timing_;
};

} // namespace trajectory
} // namespace bitsim

#endif // BITSIM_TRAJECTORY_TIME_SCALING_H

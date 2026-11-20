// Point to point motions of the spatial arm of the kinematics examples. The arm moves from one
// joint configuration to another in 2 s, timed by a cubic, a quintic and a trapezoidal time
// scaling, and the example compares their speeds, accelerations and how they start. Then it
// compares a straight line in joint space with a straight line in the world.
//
// Usage: point_to_point

#include "bitsim/bitsim_config.h"

#ifdef BITSIM_CHRONO

#include "bitsim/bitsim_consts.h"
#include "bitsim/bitsim_types.h"
#include "bitsim/kinematics/spatial_arm.h"
#include "bitsim/trajectory/time_scaling.h"
#include "bitsim/utils/io/print_utils.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace point_to_point
{

using bitsim::real_t;
using chrono::ChVector3d;
using namespace bitsim::kinematics;
using namespace bitsim::trajectory;
using namespace bitsim::utils::io;

const real_t DEG = bitsim::consts::maths::PI / 180.0;
const real_t DURATION = 2.0; // [s]
const real_t DT = 0.001;     // [s] to sample the motions

// The arm of the kinematics examples, from a pose on its right to one on its left
const SpatialArm ARM;
const std::vector<real_t> Q_START{-60.0 * DEG, 20.0 * DEG, -70.0 * DEG};
const std::vector<real_t> Q_END{70.0 * DEG, 50.0 * DEG, -30.0 * DEG};

SpatialJoints joints(const std::vector<real_t> &q) { return {q[0], q[1], q[2]}; }

real_t largest(const std::vector<real_t> &v)
{
    real_t m = 0.0;
    for (const real_t x : v)
    {
        m = std::max(m, std::abs(x));
    }
    return m;
}

void the_three_timings()
{
    print_title("1. Three ways to time the same motion");

    const CubicTimeScaling cubic(DURATION);
    const QuinticTimeScaling quintic(DURATION);
    const auto trapezoid = TrapezoidalTimeScaling::with_accel_share(DURATION, 0.25);

    std::cout << "    t [s]   cubic s   ds/dt  d2s/dt2   quintic s   ds/dt  d2s/dt2"
                 "   trapezoid s   ds/dt  d2s/dt2\n";
    for (int k = 0; k <= 8; ++k)
    {
        const real_t t = 0.25 * k;
        std::cout << std::fixed << std::setprecision(2) << std::setw(9) << t;
        for (const TimeScaling *timing :
             {static_cast<const TimeScaling *>(&cubic), static_cast<const TimeScaling *>(&quintic),
              static_cast<const TimeScaling *>(&trapezoid)})
        {
            std::cout << std::setprecision(3) << std::setw(timing == &cubic ? 10 : 12)
                      << timing->s(t) << std::setw(8) << timing->ds(t) << std::setw(9)
                      << clean(timing->dds(t));
        }
        std::cout << "\n";
    }
}

void compare_on_the_arm()
{
    print_title("2. On the arm");

    struct Candidate
    {
        std::string name;
        std::shared_ptr<const TimeScaling> timing;
    };
    const std::vector<Candidate> candidates{
        {"cubic", std::make_shared<CubicTimeScaling>(DURATION)},
        {"quintic", std::make_shared<QuinticTimeScaling>(DURATION)},
        {"trapezoid", std::make_shared<TrapezoidalTimeScaling>(
                          TrapezoidalTimeScaling::with_accel_share(DURATION, 0.25))}};

    std::cout << "  motion       top joint speed   top joint accel   accel at t = 0   top jerk\n"
              << "                       [deg/s]         [deg/s^2]        [deg/s^2]  [deg/s^3]\n";
    for (const auto &candidate : candidates)
    {
        const PointToPoint motion(Q_START, Q_END, candidate.timing);
        real_t top_speed = 0.0;
        real_t top_accel = 0.0;
        real_t top_jerk = 0.0;
        real_t last_accel = largest(motion.acceleration(0.0));
        for (real_t t = DT; t <= DURATION + 1e-9; t += DT)
        {
            top_speed = std::max(top_speed, largest(motion.velocity(t)));
            const real_t accel = largest(motion.acceleration(t));
            top_accel = std::max(top_accel, accel);
            top_jerk = std::max(top_jerk, std::abs(accel - last_accel) / DT);
            last_accel = accel;
        }
        // The jump from rest at t = 0 counts too
        top_jerk = std::max(top_jerk, largest(motion.acceleration(0.0)) / DT);

        std::cout << "  " << std::left << std::setw(11) << candidate.name << std::right
                  << std::fixed << std::setprecision(1) << std::setw(17) << top_speed / DEG
                  << std::setw(18) << top_accel / DEG << std::setw(17)
                  << largest(motion.acceleration(0.0)) / DEG << std::setw(11)
                  << std::setprecision(0) << top_jerk / DEG << "\n";
    }
    std::cout << "  (the jerk is from steps of " << DT * 1000 << " ms: a jump in acceleration "
              << "shows as a huge one)\n";
}

void joint_space_or_world()
{
    print_title("3. A straight line in joint space is not one in the world");

    // The same start and end, once with the joints moving together along a straight line in
    // joint space, once with the tip moving along a straight line in the world, by inverse
    // kinematics at every step
    const auto timing = std::make_shared<QuinticTimeScaling>(DURATION);
    const PointToPoint in_joints(Q_START, Q_END, timing);
    const ChVector3d p_start = spatial_fk(ARM, joints(Q_START)).GetPos();
    const ChVector3d p_end = spatial_fk(ARM, joints(Q_END)).GetPos();
    print_vector("tip at the start", p_start, 30);
    print_vector("tip at the end", p_end, 30);

    const auto off_line = [&](const ChVector3d &p)
    {
        const ChVector3d d = (p_end - p_start).GetNormalized();
        const ChVector3d r = p - p_start;
        return (r - d * r.Dot(d)).Length();
    };

    real_t joint_path = 0.0;
    real_t joint_off = 0.0;
    real_t world_off = 0.0;
    real_t world_top_speed = 0.0;
    ChVector3d last = p_start;
    SpatialJoints q = joints(Q_START);
    SpatialJoints q_last = q;
    for (real_t t = DT; t <= DURATION + 1e-9; t += DT)
    {
        const ChVector3d p = spatial_fk(ARM, joints(in_joints.position(t))).GetPos();
        joint_path += (p - last).Length();
        joint_off = std::max(joint_off, off_line(p));
        last = p;

        // The tip on the straight line, at the share of the way the timing gives
        const ChVector3d target = p_start + (p_end - p_start) * timing->s(t);
        const auto result = numerical_ik(ARM, target, q, 0.05);
        q = result.q;
        world_off = std::max(world_off, off_line(spatial_fk(ARM, q).GetPos()));
        for (int j = 0; j < 3; ++j)
        {
            world_top_speed = std::max(world_top_speed, std::abs(q[j] - q_last[j]) / DT);
        }
        q_last = q;
    }
    print_number("straight line [m]", (p_end - p_start).Length(), 30);
    print_number("joint space: tip travels [m]", joint_path, 30);
    print_number("joint space: off the line [m]", joint_off, 30);
    print_error("world: off the line [m]", world_off, 30);
    print_number("world: top joint speed [deg/s]", world_top_speed / DEG, 30);
}

} // namespace point_to_point

int main()
{
    point_to_point::the_three_timings();
    point_to_point::compare_on_the_arm();
    point_to_point::joint_space_or_world();
    return 0;
}

#else

#include <iostream>

int main()
{
    std::cerr << "point_to_point needs bitsim configured with ENABLE_CHRONO=ON" << std::endl;
    return 1;
}

#endif

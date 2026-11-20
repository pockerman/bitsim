#include "bitsim/planning/rrt.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace bitsim
{
namespace planning
{

namespace
{
real_t distance(const RRT::Point &a, const RRT::Point &b)
{
    return std::hypot(b[0] - a[0], b[1] - a[1]);
}
} // namespace

RRT::RRT(const Point &lower, const Point &upper, FreeTest is_free, const Parameters &parameters,
         uint_t seed)
    : lower_(lower), upper_(upper), is_free_(std::move(is_free)), p_(parameters),
      rng_(static_cast<std::mt19937::result_type>(seed))
{
    if (upper[0] <= lower[0] || upper[1] <= lower[1])
    {
        throw std::invalid_argument("RRT: upper must be above and to the right of lower");
    }
    if (p_.step <= 0.0 || p_.collision_step <= 0.0)
    {
        throw std::invalid_argument("RRT: the steps must be positive");
    }
}

bool RRT::plan(const Point &start, const Point &goal)
{
    tree_.clear();
    reached_ = false;
    n_iterations_ = 0;
    if (!is_free_(start) || !is_free_(goal))
    {
        return false;
    }
    tree_.push_back({start, 0});

    std::uniform_real_distribution<real_t> unit(0.0, 1.0);
    std::uniform_real_distribution<real_t> x(lower_[0], upper_[0]);
    std::uniform_real_distribution<real_t> y(lower_[1], upper_[1]);

    for (; n_iterations_ < p_.max_iterations; ++n_iterations_)
    {
        const Point target = unit(rng_) < p_.goal_bias ? goal : Point{x(rng_), y(rng_)};
        const uint_t near = nearest_(target);
        const Point &from = tree_[near].p;

        const real_t d = distance(from, target);
        if (d < 1.0e-12)
        {
            continue;
        }
        const real_t s = std::min(p_.step, d) / d;
        const Point to{from[0] + s * (target[0] - from[0]), from[1] + s * (target[1] - from[1])};
        if (!is_segment_free(from, to))
        {
            continue;
        }
        tree_.push_back({to, near});

        if (distance(to, goal) <= p_.goal_radius && is_segment_free(to, goal))
        {
            tree_.push_back({goal, tree_.size() - 1});
            goal_node_ = tree_.size() - 1;
            reached_ = true;
            ++n_iterations_;
            return true;
        }
    }
    return false;
}

std::vector<RRT::Point> RRT::path() const
{
    std::vector<Point> result;
    if (!reached_)
    {
        return result;
    }
    for (uint_t n = goal_node_;; n = tree_[n].parent)
    {
        result.push_back(tree_[n].p);
        if (n == 0)
        {
            break;
        }
    }
    std::reverse(result.begin(), result.end());
    return result;
}

bool RRT::is_segment_free(const Point &a, const Point &b) const
{
    const real_t d = distance(a, b);
    const auto n = static_cast<uint_t>(std::ceil(d / p_.collision_step));
    for (uint_t k = 0; k <= n; ++k)
    {
        const real_t t = n == 0 ? 0.0 : static_cast<real_t>(k) / n;
        if (!is_free_({a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])}))
        {
            return false;
        }
    }
    return true;
}

std::vector<RRT::Point> RRT::shortcut(const std::vector<Point> &path) const
{
    if (path.size() < 3)
    {
        return path;
    }
    std::vector<Point> result{path.front()};
    std::size_t i = 0;
    while (i + 1 < path.size())
    {
        std::size_t j = path.size() - 1;
        while (j > i + 1 && !is_segment_free(path[i], path[j]))
        {
            --j;
        }
        result.push_back(path[j]);
        i = j;
    }
    return result;
}

real_t RRT::length(const std::vector<Point> &path)
{
    real_t total = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i)
    {
        total += distance(path[i - 1], path[i]);
    }
    return total;
}

uint_t RRT::nearest_(const Point &p) const
{
    uint_t best = 0;
    real_t best_d = distance(tree_[0].p, p);
    for (uint_t i = 1; i < tree_.size(); ++i)
    {
        const real_t d = distance(tree_[i].p, p);
        if (d < best_d)
        {
            best = i;
            best_d = d;
        }
    }
    return best;
}

} // namespace planning
} // namespace bitsim

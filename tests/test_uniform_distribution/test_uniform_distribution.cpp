#include "bitsim/bitsim_types.h"
#include "bitsim/maths/statistics/distributions/uniform_distribution.h"

#include <gtest/gtest.h>

#include <utility>
#include <set>
#include <vector>

namespace
{
using bitsim::real_t;
using bitsim::uint_t;
using bitsim::maths::stats::UniformDist;
using bitsim::maths::stats::UniformWeightedDist;
} // namespace

TEST(TestUniformDist, SamplesStayInTheRange)
{
    UniformDist<real_t> dist(-1.0, 2.0);
    EXPECT_DOUBLE_EQ(dist.range().first, -1.0);
    EXPECT_DOUBLE_EQ(dist.range().second, 2.0);
    for (uint_t s = 0; s < 200; ++s)
    {
        const real_t x = dist.sample(s);
        EXPECT_GE(x, -1.0);
        EXPECT_LT(x, 2.0);
    }

    // Integers: both ends of the range are included, and every value turns up
    UniformDist<uint_t> dice(std::make_pair<uint_t, uint_t>(1, 6));
    std::set<uint_t> seen;
    for (uint_t s = 0; s < 500; ++s)
    {
        seen.insert(dice.sample(s));
    }
    EXPECT_EQ(seen, (std::set<uint_t>{1, 2, 3, 4, 5, 6}));
}

TEST(TestUniformDist, TheSameSeedGivesTheSameSample)
{
    UniformDist<real_t> a(0.0, 1.0);
    UniformDist<real_t> b(0.0, 1.0);
    EXPECT_DOUBLE_EQ(a.sample(42), b.sample(42));
    EXPECT_NE(a.sample(1), a.sample(2));
}

TEST(TestUniformWeightedDist, ProbabilitiesFollowTheWeights)
{
    UniformWeightedDist<uint_t> dist({1.0, 3.0});
    const auto p = dist.probabilities();
    ASSERT_EQ(p.size(), 2u);
    EXPECT_DOUBLE_EQ(p[0], 0.25);
    EXPECT_DOUBLE_EQ(p[1], 0.75);

    // A zero weight is never drawn
    UniformWeightedDist<uint_t> only_last(std::vector<real_t>{0.0, 0.0, 1.0});
    for (uint_t s = 0; s < 50; ++s)
    {
        EXPECT_EQ(only_last.sample(s), 2u);
    }
}

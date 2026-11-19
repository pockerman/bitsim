#include "bitrl/rigid_bodies/body_rotation.h"
#include "bitrl/rigid_bodies/body_translation.h"

#include <gtest/gtest.h>

#include <sstream>
#include <stdexcept>

using bitrl::rigid_bodies::RBRotation;
using bitrl::rigid_bodies::RBTranslation;

TEST(TestRigidBodyTypes, HoldThreeValues)
{
    const RBTranslation t{1.0, 2.0, 3.0};
    EXPECT_DOUBLE_EQ(t.x, 1.0);
    EXPECT_DOUBLE_EQ(t.y, 2.0);
    EXPECT_DOUBLE_EQ(t.z, 3.0);

    const RBRotation r;
    EXPECT_DOUBLE_EQ(r.x, 0.0);
    EXPECT_DOUBLE_EQ(r.y, 0.0);
    EXPECT_DOUBLE_EQ(r.z, 0.0);

    std::ostringstream out;
    out << t << RBRotation{0.5, -0.5, 0.25};
    EXPECT_EQ(out.str(), "1, 2, 3\n0.5, -0.5, 0.25\n");
}

TEST(TestRigidBodyTypes, NeedExactlyThreeValues)
{
    EXPECT_THROW(RBTranslation({1.0}), std::invalid_argument);
    EXPECT_THROW(RBRotation({1.0, 2.0, 3.0, 4.0}), std::invalid_argument);
}

#include "util.hh"
#include <gtest/gtest.h>

TEST(OppositeSigns, Tests)
{
    ASSERT_TRUE(OppositeSigns(-1.4, 3.88));
    ASSERT_FALSE(OppositeSigns(-4.1, 0));
    ASSERT_FALSE(OppositeSigns(-10.3, -2.5));
    ASSERT_FALSE(OppositeSigns(0, 0));
    ASSERT_FALSE(OppositeSigns(4.12, 2.423));
}

TEST(GetPerpendicular, Tests)
{
    ImplicitTriangulation::Vector3D vec;
    srand(time(NULL));
    vec[0] = 10;
    vec[1] = rand() % 100;
    vec[2] = rand() % 100;
    const auto vecLen = vec.length();
    const auto perp = GetPerpendicular(vec);
    EXPECT_NEAR(0, perp.dot(vec), 1e-11);
    EXPECT_NEAR(vecLen, perp.length(), 1e-11);
    const auto rotatedPerp = Rotate(vec.normalized(), M_PI / 6.0, perp);
    EXPECT_NEAR(0, rotatedPerp.dot(vec), 1e-11);
}
#include "gtest/gtest.h"
#include "roboteam_utils/Mathematics.h"

namespace rtt {

TEST(MathematicsTest, toDegrees) {
    // Positive test cases
    EXPECT_DOUBLE_EQ(toDegrees(0), 0);
    EXPECT_DOUBLE_EQ(toDegrees(M_PI), 180);
    EXPECT_DOUBLE_EQ(toDegrees(2 * M_PI), 360);
    EXPECT_DOUBLE_EQ(toDegrees(3 * M_PI), 540);

    // Negative test cases
    EXPECT_DOUBLE_EQ(toDegrees(-M_PI), -180);
    EXPECT_DOUBLE_EQ(toDegrees(-2 * M_PI), -360);
    EXPECT_DOUBLE_EQ(toDegrees(-3 * M_PI), -540);

    // Floating point comparison
    const double tolerance = 1e-5;
    EXPECT_NEAR(toDegrees(M_PI / 4), 45, tolerance);
    EXPECT_NEAR(toDegrees(M_PI / 2), 90, tolerance);
    EXPECT_NEAR(toDegrees(3 * M_PI / 2), 270, tolerance);
}

TEST(MathematicsTest, toRadians) {
    // Positive test cases
    EXPECT_DOUBLE_EQ(toRadians(0), 0);
    EXPECT_DOUBLE_EQ(toRadians(180), M_PI);
    EXPECT_DOUBLE_EQ(toRadians(360), 2 * M_PI);
    EXPECT_DOUBLE_EQ(toRadians(540), 3 * M_PI);

    // Negative test cases
    EXPECT_DOUBLE_EQ(toRadians(-180), -M_PI);
    EXPECT_DOUBLE_EQ(toRadians(-360), -2 * M_PI);
    EXPECT_DOUBLE_EQ(toRadians(-540), -3 * M_PI);

    // Floating point comparison
    const double tolerance = 1e-5;
    EXPECT_NEAR(toRadians(45), M_PI / 4, tolerance);
    EXPECT_NEAR(toRadians(90), M_PI / 2, tolerance);
    EXPECT_NEAR(toRadians(270), 3 * M_PI / 2, tolerance);
}

TEST(MathematicsTest, cleanAngle) {
    EXPECT_DOUBLE_EQ(cleanAngle(0), 0);
    EXPECT_DOUBLE_EQ(cleanAngle(2 * M_PI), 0);
    EXPECT_DOUBLE_EQ(cleanAngle(3 * M_PI), -M_PI);
    EXPECT_DOUBLE_EQ(cleanAngle(-M_PI), M_PI);
    EXPECT_DOUBLE_EQ(cleanAngle(-2 * M_PI), 0);
    EXPECT_DOUBLE_EQ(cleanAngle(-3 * M_PI), M_PI);
}

}  // namespace rtt
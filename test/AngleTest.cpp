#include <gtest/gtest.h>
#include "roboteam_utils/Angle.h"

namespace rtt {
double EPSILON = 1e-6;          // Difference that is used to check whether the actual and expected value are similar.
double SUPREME_EPSILON = 1e-9;  // Small value that is used to create test cases

TEST(AngleTests, rescale) {
    /* Apply equivalence partitioning as testing method (https://en.wikipedia.org/wiki/Equivalence_partitioning) with the following partitions:
     * angle < -PI, -PI <= angle < 0, angle = 0, 0 < angle < PI, angle >= PI
     * and extend these partitions with boundary value analysis. We do not test angle = -FLT_MAX or angle = FLT_MAX, since we expect things to go wrong for these test cases, but
     * we test other big positive and negative value instead. */

    // Partition tests for: angle < -PI
    double testAngle = -372349.9654;  // A random large negative number
    double expectedRescale = -2.120912;
    double actualRescale = Angle(testAngle);
    ASSERT_TRUE(fabs(actualRescale - expectedRescale) < EPSILON);
    testAngle = -M_PI - SUPREME_EPSILON;
    expectedRescale = M_PI;
    actualRescale = Angle(testAngle);
    ASSERT_TRUE(fabs(actualRescale - expectedRescale) < EPSILON);

    // Partition tests for: -PI <= angle < 0
    testAngle = -M_PI;
    expectedRescale = -M_PI;
    actualRescale = Angle(testAngle);
    ASSERT_TRUE(fabs(actualRescale - expectedRescale) < EPSILON);
    testAngle = -M_PI / 3;
    expectedRescale = -M_PI / 3;
    actualRescale = Angle(testAngle);
    ASSERT_TRUE(fabs(actualRescale - expectedRescale) < EPSILON);
    testAngle = -SUPREME_EPSILON;
    expectedRescale = 0.0;
    actualRescale = Angle(testAngle);
    ASSERT_TRUE(fabs(actualRescale - expectedRescale) < EPSILON);

    // Partition tests for: angle = 0
    testAngle = 0.0;
    expectedRescale = 0.0;
    actualRescale = Angle(testAngle);
    ASSERT_TRUE(fabs(actualRescale - expectedRescale) < EPSILON);

    // Partition tests for: 0 < angle < PI
    testAngle = SUPREME_EPSILON;
    expectedRescale = 0.0;
    actualRescale = Angle(testAngle);
    ASSERT_TRUE(fabs(actualRescale - expectedRescale) < EPSILON);
    testAngle = 3 * M_PI / 4;
    expectedRescale = 3 * M_PI / 4;
    actualRescale = Angle(testAngle);
    ASSERT_TRUE(fabs(actualRescale - expectedRescale) < EPSILON);
    testAngle = M_PI - SUPREME_EPSILON;
    expectedRescale = M_PI;
    actualRescale = Angle(testAngle);
    ASSERT_TRUE(fabs(actualRescale - expectedRescale) < EPSILON);

    // Partition tests for: angle >= PI
    testAngle = M_PI;
    expectedRescale = -M_PI;
    actualRescale = Angle(testAngle);
    ASSERT_TRUE(fabs(actualRescale - expectedRescale) < EPSILON);
    testAngle = 20128.1969;  // A random large positive number
    expectedRescale = -3.128824207;
    actualRescale = Angle(testAngle);
    ASSERT_TRUE(fabs(actualRescale - expectedRescale) < EPSILON);
}

TEST(AngleTests, rotateDirectionANDshortestAngleDiff) {
    /* Apply equivalence partitioning as testing method (https://en.wikipedia.org/wiki/Equivalence_partitioning) with the following partitions on the angle difference:
     * 2 * PI < angle_diff <= -PI, -PI < angle_diff <= 0, 0 < angle_diff <= PI, PI < angle_diff < 2 * PI
     * and extend these partitions with boundary value analysis. */

    // Partition test for: 2 * PI < angle_diff <= -PI
    Angle angle1 = Angle(-M_PI);
    Angle angle2 = Angle(M_PI - SUPREME_EPSILON);
    double expectedShortestDistance = 0.0;
    double actualShortestDistance = angle1.shortestAngleDiff(angle2);
    ASSERT_TRUE(fabs(actualShortestDistance - expectedShortestDistance) < EPSILON);
    ASSERT_FALSE(angle1.rotateDirection(angle2));
    angle1 = Angle(-3 * M_PI / 5);
    angle2 = Angle(4 * M_PI / 5);
    expectedShortestDistance = 3 * M_PI / 5;
    actualShortestDistance = angle1.shortestAngleDiff(angle2);
    ASSERT_TRUE(fabs(actualShortestDistance - expectedShortestDistance) < EPSILON);
    ASSERT_FALSE(angle1.rotateDirection(angle2));
    angle1 = Angle(-3 * M_PI / 4 - SUPREME_EPSILON);
    angle2 = Angle(M_PI / 4);
    expectedShortestDistance = M_PI;
    actualShortestDistance = angle1.shortestAngleDiff(angle2);
    ASSERT_TRUE(fabs(actualShortestDistance - expectedShortestDistance) < EPSILON);
    ASSERT_FALSE(angle1.rotateDirection(angle2));

    // Partition test for: -PI < angle_diff <= 0
    angle1 = Angle(-M_PI / 3 + SUPREME_EPSILON);
    angle2 = Angle(2 * M_PI / 3);
    expectedShortestDistance = M_PI;
    actualShortestDistance = angle1.shortestAngleDiff(angle2);
    ASSERT_TRUE(fabs(actualShortestDistance - expectedShortestDistance) < EPSILON);
    ASSERT_TRUE(angle1.rotateDirection(angle2));
    angle1 = Angle(M_PI / 4);
    angle2 = Angle(3 * M_PI / 4);
    expectedShortestDistance = M_PI / 2;
    actualShortestDistance = angle1.shortestAngleDiff(angle2);
    ASSERT_TRUE(fabs(actualShortestDistance - expectedShortestDistance) < EPSILON);
    ASSERT_TRUE(angle1.rotateDirection(angle2));
    angle1 = Angle(-M_PI / 6);
    angle2 = Angle(-M_PI / 6 + SUPREME_EPSILON);
    expectedShortestDistance = 0.0;
    actualShortestDistance = angle1.shortestAngleDiff(angle2);
    ASSERT_TRUE(fabs(actualShortestDistance - expectedShortestDistance) < EPSILON);
    ASSERT_TRUE(angle1.rotateDirection(angle2));

    // Partition test for: 0 < angle_diff <= PI
    angle1 = Angle(5 * M_PI / 6);
    angle2 = Angle(5 * M_PI / 6 - SUPREME_EPSILON);
    expectedShortestDistance = 0.0;
    actualShortestDistance = angle1.shortestAngleDiff(angle2);
    ASSERT_TRUE(fabs(actualShortestDistance - expectedShortestDistance) < EPSILON);
    ASSERT_FALSE(angle1.rotateDirection(angle2));
    angle1 = Angle(3 * M_PI / 8);
    angle2 = Angle(M_PI / 8);
    expectedShortestDistance = M_PI / 4;
    actualShortestDistance = angle1.shortestAngleDiff(angle2);
    ASSERT_TRUE(fabs(actualShortestDistance - expectedShortestDistance) < EPSILON);
    ASSERT_FALSE(angle1.rotateDirection(angle2));
    angle1 = Angle(M_PI / 9 - SUPREME_EPSILON);
    angle2 = Angle(-8 * M_PI / 9);
    expectedShortestDistance = M_PI;
    actualShortestDistance = angle1.shortestAngleDiff(angle2);
    ASSERT_TRUE(fabs(actualShortestDistance - expectedShortestDistance) < EPSILON);
    ASSERT_FALSE(angle1.rotateDirection(angle2));

    // Partition test for: PI < angle_diff < 2 * PI
    angle1 = Angle(5 * M_PI / 7 + SUPREME_EPSILON);
    angle2 = Angle(-2 * M_PI / 7);
    expectedShortestDistance = M_PI;
    actualShortestDistance = angle1.shortestAngleDiff(angle2);
    ASSERT_TRUE(fabs(actualShortestDistance - expectedShortestDistance) < EPSILON);
    ASSERT_TRUE(angle1.rotateDirection(angle2));
    angle1 = Angle(3 * M_PI / 4);
    angle2 = Angle(-M_PI / 2);
    expectedShortestDistance = 3 * M_PI / 4;
    actualShortestDistance = angle1.shortestAngleDiff(angle2);
    ASSERT_TRUE(fabs(actualShortestDistance - expectedShortestDistance) < EPSILON);
    ASSERT_TRUE(angle1.rotateDirection(angle2));
    angle1 = Angle(M_PI - SUPREME_EPSILON);
    angle2 = Angle(-M_PI);
    expectedShortestDistance = 0.0;
    actualShortestDistance = angle1.shortestAngleDiff(angle2);
    ASSERT_TRUE(fabs(actualShortestDistance - expectedShortestDistance) < EPSILON);
    ASSERT_TRUE(angle1.rotateDirection(angle2));
}

TEST(AngleTests, toVector2) {
    /* Apply equivalence partitioning as testing method (https://en.wikipedia.org/wiki/Equivalence_partitioning) with the following partitions on the Vector2 length:
     * 0 <= length <= 1, length > 1
     * and extend these partitions with boundary value analysis. We do not test length = FLT_MAX, since we expect things to go wrong for this test case, but we test another
     * big positive length instead. Moreover for all these cases we test 3 different angles: a horizontal angle (either 0 or -PI radian angle), a vertical angle (either PI/2 or
     * -PI/2 radian angle) and an arbitrary other angle. Since we mostly expect errors to happen for the horizontal & vertical angle, because the respective y and x coordinates are
     * zero for these angles. So division by zero might happen. */

    // The length 0.0 test cases
    double length = 0.0;
    Angle angle = Angle(0.0);
    Vector2 expectedVector = Vector2(0.0, 0.0);
    Vector2 actualVector = angle.toVector2(length);
    ASSERT_EQ(expectedVector, actualVector);
    angle = Angle(-M_PI / 2);
    actualVector = angle.toVector2(length);
    ASSERT_EQ(expectedVector, actualVector);
    angle = Angle(-7 * M_PI / 11);
    actualVector = angle.toVector2(length);
    ASSERT_EQ(expectedVector, actualVector);

    // The length 1/3 test cases
    length = 1.0 / 3.0;
    angle = Angle(-M_PI);
    expectedVector = Vector2(-length, 0.0);
    actualVector = angle.toVector2(length);
    ASSERT_EQ(expectedVector, actualVector);
    angle = Angle(M_PI / 2);
    expectedVector = Vector2(0.0, length);
    actualVector = angle.toVector2(length);
    ASSERT_EQ(expectedVector, actualVector);
    angle = Angle(-2 * M_PI / 6);
    expectedVector = Vector2(0.5, -0.5 * sqrt(3)) * length;
    actualVector = angle.toVector2(length);
    ASSERT_EQ(expectedVector, actualVector);

    // The length 1.0 test cases
    length = 1.0;
    angle = Angle(0.0);
    expectedVector = Vector2(1.0, 0.0);
    actualVector = angle.toVector2(length);
    ASSERT_EQ(expectedVector, actualVector);
    angle = Angle(M_PI / 2);
    expectedVector = Vector2(0.0, 1.0);
    actualVector = angle.toVector2(length);
    ASSERT_EQ(expectedVector, actualVector);
    angle = Angle(3 * M_PI / 4);
    expectedVector = Vector2(-0.5 * sqrt(2), 0.5 * sqrt(2));
    actualVector = angle.toVector2(length);
    ASSERT_EQ(expectedVector, actualVector);

    // The length 1e9 test cases
    length = 1e9;
    angle = Angle(-M_PI);
    expectedVector = Vector2(-length, 0.0);
    actualVector = angle.toVector2(length);
    ASSERT_EQ(expectedVector, actualVector);
    angle = Angle(-M_PI / 2);
    expectedVector = Vector2(0.0, -length);
    actualVector = angle.toVector2(length);
    ASSERT_EQ(expectedVector, actualVector);
    angle = Angle(M_PI / 6);
    expectedVector = Vector2(0.5 * sqrt(3), 0.5) * length;
    actualVector = angle.toVector2(length);
    ASSERT_EQ(expectedVector, actualVector);
}

}  // namespace rtt
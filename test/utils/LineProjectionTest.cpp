//
// Created by Haico Dorenbos on 10-06-2020.
//
#include <gtest/gtest.h>
#include <math.h>
#include <roboteam_utils/HalfLine.h>
#include "roboteam_utils/Line.h"
#include "roboteam_utils/LineSegment.h"
#include "roboteam_utils/Vector2.h"

const double EPSILON = 1e-2; // Small difference that is still significant enough to really change a Vector2 position.

using namespace rtt;

/* In this test class we test the isOnLine functions and the projection methods in different line classes: Line, HalfLine and LineSegment. For these methods we want to cover 3 type
 * of lines: vertical line (does not change in x direction), horizontal line (does not change in y direction), arbitrary other line (does change in both x and y direction). This is
 * to test cases where there could be a possibility of division by zero. Furthermore we test two different cases, where the point is on the Line interpretation and where the point
 * is not on the Line interpretation. As test method we use equivalence partitioning, based on the relative position of the Line interpretation projection:
 * t < 0, 0 <= t <= 1, 1 < t. We extend these test cases with boundary value analysis, but for t is (minus) infinity we use a very small and very large number (we expect
 * intersections to fail for FLT_MAX and -FLT_MAX). Also we create a 2 test case where we do a projection of a point on itself as LineSegment and a projection of a point on another
 * point which is a LineSegment.
 */
TEST(LineProjectionTests, NotOnLine1) {
    // Test case with a horizontal line, where the point is not on the Line. And the projection position on the Line interpretation is at t = -LARGE.
    Vector2 start(-7.0, 4.0), end(-3.0, 4.0), point(-1000000.0, -1.0);
    Line line(start, end); HalfLine halfLine(start, end); LineSegment lineSegment(start, end);
    Vector2 expected = {-1000000.0, 4.0};
    Vector2 actual = line.project(point);
    EXPECT_FALSE(line.isOnLine(point));
    EXPECT_EQ(expected, actual);
    expected = {-7.0, 4.0};
    actual = halfLine.project(point);
    EXPECT_EQ(expected, actual);
    expected = {-7.0, 4.0};
    actual = lineSegment.project(point);
    EXPECT_FALSE(lineSegment.isOnLine(point));
    EXPECT_EQ(expected, actual);
}

TEST(LineProjectionTests, NotOnLine2) {
    // Test case with a vertical line, where the point is not on the Line. And the projection position on the Line interpretation is at t = -EPSILON.
    Vector2 start(5.0, 2.0), end(5.0, -2.0), point(-1000.0, 2.0 + EPSILON);
    Line line(start, end); HalfLine halfLine(start, end); LineSegment lineSegment(start, end);
    Vector2 expected = {5.0, 2.0 + EPSILON};
    Vector2 actual = line.project(point);
    EXPECT_FALSE(line.isOnLine(point));
    EXPECT_EQ(expected, actual);
    expected = {5.0, 2.0};
    actual = halfLine.project(point);
    EXPECT_EQ(expected, actual);
    expected = {5.0, 2.0};
    actual = lineSegment.project(point);
    EXPECT_FALSE(lineSegment.isOnLine(point));
    EXPECT_EQ(expected, actual);
}

TEST(LineProjectionTests, NotOnLine3) {
    // Test case with an arbitrary line, where the point is not on the Line. And the projection position on the Line interpretation is at t = 0.
    Vector2 start(-4.0, -8.0), end(-8.0, -4.0), point(-2.0, -6.0);
    Line line(start, end); HalfLine halfLine(start, end); LineSegment lineSegment(start, end);
    Vector2 expected = {-4.0, -8.0};
    Vector2 actual = line.project(point);
    EXPECT_FALSE(line.isOnLine(point));
    EXPECT_EQ(expected, actual);
    expected = {-4.0, -8.0};
    actual = halfLine.project(point);
    EXPECT_EQ(expected, actual);
    expected = {-4.0, -8.0};
    actual = lineSegment.project(point);
    EXPECT_FALSE(lineSegment.isOnLine(point));
    EXPECT_EQ(expected, actual);
}

TEST(LineProjectionTests, NotOnLine4) {
    // Test case with a vertical line, where the point is not on the Line. And the projection position on the Line interpretation is at 0 < t < 1.
    Vector2 start(5.0, -1.0), end(5.0, 6.0), point(0.0, 0.0);
    Line line(start, end); HalfLine halfLine(start, end); LineSegment lineSegment(start, end);
    Vector2 expected = {5.0, 0.0};
    Vector2 actual = line.project(point);
    EXPECT_FALSE(line.isOnLine(point));
    EXPECT_EQ(expected, actual);
    expected = {5.0, 0.0};
    actual = halfLine.project(point);
    EXPECT_EQ(expected, actual);
    expected = {5.0, 0.0};
    actual = lineSegment.project(point);
    EXPECT_FALSE(lineSegment.isOnLine(point));
    EXPECT_EQ(expected, actual);
}

TEST(LineProjectionTests, NotOnLine5) {
    // Test case with a horizontal line, where the point is not on the Line. And the projection position on the Line interpretation is at t = 1.
    Vector2 start(7.0, -3.0), end(4.0, -3.0), point(6.0, -3.0 - EPSILON);
    Line line(start, end); HalfLine halfLine(start, end); LineSegment lineSegment(start, end);
    Vector2 expected = {6.0, -3.0};
    Vector2 actual = line.project(point);
    EXPECT_FALSE(line.isOnLine(point));
    EXPECT_EQ(expected, actual);
    expected = {6.0, -3.0};
    actual = halfLine.project(point);
    EXPECT_EQ(expected, actual);
    expected = {6.0, -3.0};
    actual = lineSegment.project(point);
    EXPECT_FALSE(lineSegment.isOnLine(point));
    EXPECT_EQ(expected, actual);
}

TEST(LineProjectionTests, NotOnLine6) {
    // Test case with an arbitrary line, where the point is not on the Line. And the projection position on the Line interpretation is at t = 1 + EPSILON.
    Vector2 start(-10.0, -4.0), end(-2.0 - EPSILON, -8.0 + EPSILON / 2), point(6.0, 8.0);
    Line line(start, end); HalfLine halfLine(start, end); LineSegment lineSegment(start, end);
    Vector2 expected = {-2.0, -8.0};
    Vector2 actual = line.project(point);
    EXPECT_FALSE(line.isOnLine(point));
    EXPECT_EQ(expected, actual);
    expected = {-2.0, -8.0};
    actual = halfLine.project(point);
    EXPECT_EQ(expected, actual);
    expected = {-2.0 - EPSILON, -8.0 + EPSILON / 2};
    actual = lineSegment.project(point);
    EXPECT_FALSE(lineSegment.isOnLine(point));
    EXPECT_EQ(expected, actual);
}

TEST(LineProjectionTests, NotOnLine7) {
    // Test case with an horizontal line, where the point is not on the Line. And the projection position on the Line interpretation is at t = LARGE.
    Vector2 start(-10.0, 4.0), end(-3.0, 4.0), point(1e6, 1e6);
    Line line(start, end); HalfLine halfLine(start, end); LineSegment lineSegment(start, end);
    Vector2 expected = {1e6, 4.0};
    Vector2 actual = line.project(point);
    EXPECT_FALSE(line.isOnLine(point));
    EXPECT_EQ(expected, actual);
    expected = {1e6, 4.0};
    actual = halfLine.project(point);
    EXPECT_EQ(expected, actual);
    expected = {-3.0, 4.0};
    actual = lineSegment.project(point);
    EXPECT_FALSE(lineSegment.isOnLine(point));
    EXPECT_EQ(expected, actual);
}

TEST(LineProjectionTests, OnLine1) {
    // Test case with a vertical line, where the point is on the Line. And the projection position on the Line interpretation is at t = -LARGE.
    Vector2 start(0.0, -8.0), end(0.0, -8.0 - EPSILON), point(0.0, 0.0);
    Line line(start, end); HalfLine halfLine(start, end); LineSegment lineSegment(start, end);
    Vector2 expected = {0.0, 0.0};
    Vector2 actual = line.project(point);
    EXPECT_TRUE(line.isOnLine(point));
    EXPECT_EQ(expected, actual);
    expected = {0.0, -8.0};
    actual = halfLine.project(point);
    EXPECT_EQ(expected, actual);
    expected = {0.0, -8.0};
    actual = lineSegment.project(point);
    EXPECT_FALSE(lineSegment.isOnLine(point));
    EXPECT_EQ(expected, actual);
}

TEST(LineProjectionTests, OnLine2) {
    // Test case with an arbitrary line, where the point is on the Line. And the projection position on the Line interpretation is at t = -EPSILON.
    Vector2 start(4.0, -1.0), end(104.0, -1000001.0), point(4.0 - EPSILON, -1.0 + 10000 * EPSILON);
    Line line(start, end); HalfLine halfLine(start, end); LineSegment lineSegment(start, end);
    Vector2 expected = {4.0 - EPSILON, -1.0 + 10000 * EPSILON};
    Vector2 actual = line.project(point);
    EXPECT_TRUE(line.isOnLine(point));
    EXPECT_EQ(expected, actual);
    expected = {4.0, -1.0};
    actual = halfLine.project(point);
    EXPECT_EQ(expected, actual);
    expected = {4.0, -1.0};
    actual = lineSegment.project(point);
    EXPECT_FALSE(lineSegment.isOnLine(point));
    EXPECT_EQ(expected, actual);
}

TEST(LineProjectionTests, OnLine3) {
    // Test case with a vertical line, where the point is on the Line. And the projection position on the Line interpretation is at t = 0.
    Vector2 start(1000.0, -2999.0), end(1000.0, -3000.0), point(1000.0, -2999.0);
    Line line(start, end); HalfLine halfLine(start, end); LineSegment lineSegment(start, end);
    Vector2 expected = {1000.0, -2999.0};
    Vector2 actual = line.project(point);
    EXPECT_TRUE(line.isOnLine(point));
    EXPECT_EQ(expected, actual);
    expected = {1000.0, -2999.0};
    actual = halfLine.project(point);
    EXPECT_EQ(expected, actual);
    expected = {1000.0, -2999.0};
    actual = lineSegment.project(point);
    EXPECT_TRUE(lineSegment.isOnLine(point));
    EXPECT_EQ(expected, actual);
}

TEST(LineProjectionTests, OnLine4) {
    // Test case with a horizontal line, where the point is on the Line. And the projection position on the Line interpretation is at 0 < t < 1.
    Vector2 start(5.0, 5.0), end(-5.0, 5.0), point(-1.0, 5.0);
    Line line(start, end); HalfLine halfLine(start, end); LineSegment lineSegment(start, end);
    Vector2 expected = {-1.0, 5.0};
    Vector2 actual = line.project(point);
    EXPECT_TRUE(line.isOnLine(point));
    EXPECT_EQ(expected, actual);
    expected = {-1.0, 5.0};
    actual = halfLine.project(point);
    EXPECT_EQ(expected, actual);
    expected = {-1.0, 5.0};
    actual = lineSegment.project(point);
    EXPECT_TRUE(lineSegment.isOnLine(point));
    EXPECT_EQ(expected, actual);
}

TEST(LineProjectionTests, OnLine5) {
    // Test case with an arbitrary line, where the point is on the Line. And the projection position on the Line interpretation is t = 1.
    Vector2 start(-3.0, 10.0), end(1e6, 9.0), point(1e6, 9.0);
    Line line(start, end); HalfLine halfLine(start, end); LineSegment lineSegment(start, end);
    Vector2 expected = {1e6, 9.0};
    Vector2 actual = line.project(point);
    EXPECT_TRUE(line.isOnLine(point));
    EXPECT_EQ(expected, actual);
    expected = {1e6, 9.0};
    actual = halfLine.project(point);
    EXPECT_EQ(expected, actual);
    expected = {1e6, 9.0};
    actual = lineSegment.project(point);
    EXPECT_TRUE(lineSegment.isOnLine(point));
    EXPECT_EQ(expected, actual);
}

TEST(LineProjectionTests, OnLine6) {
    // Test case with a horizontal line, where the point is on the Line. And the projection position on the Line interpretation is t = 1 + EPSILON.
    Vector2 start(-1e6, -1e6), end(1e6, -1e6), point(1000001.0, -1e6);
    Line line(start, end); HalfLine halfLine(start, end); LineSegment lineSegment(start, end);
    Vector2 expected = {1000001.0, -1e6};
    Vector2 actual = line.project(point);
    EXPECT_TRUE(line.isOnLine(point));
    EXPECT_EQ(expected, actual);
    expected = {1000001.0, -1e6};
    actual = halfLine.project(point);
    EXPECT_EQ(expected, actual);
    expected = {1e6, -1e6};
    actual = lineSegment.project(point);
    EXPECT_FALSE(lineSegment.isOnLine(point));
    EXPECT_EQ(expected, actual);
}

TEST(LineProjectionTests, OnLine7) {
    // Test case with a vertical line, where the point is on the Line. And the projection position on the Line interpretation is t = LARGE.
    Vector2 start(-5.0, EPSILON), end(-5.0, -EPSILON), point(-5.0, -1e9);
    Line line(start, end); HalfLine halfLine(start, end); LineSegment lineSegment(start, end);
    Vector2 expected = {-5.0, -1e9};
    Vector2 actual = line.project(point);
    EXPECT_TRUE(line.isOnLine(point));
    EXPECT_EQ(expected, actual);
    expected = {-5.0, -1e9};
    actual = halfLine.project(point);
    EXPECT_EQ(expected, actual);
    expected = {-5.0, -EPSILON};
    actual = lineSegment.project(point);
    EXPECT_FALSE(lineSegment.isOnLine(point));
    EXPECT_EQ(expected, actual);
}

TEST(LineProjectionTests, Point1) {
    // Test case with 2 equal points, where one of them is treated as a LineSegment.
    Vector2 start(0.0, 0.0), end(0.0, 0.0), point(0.0, 0.0);
    LineSegment lineSegment(start, end);
    Vector2 expected = {0.0, 0.0};
    Vector2 actual = lineSegment.project(point);
    EXPECT_TRUE(lineSegment.isOnLine(point));
    EXPECT_EQ(expected, actual);
}

TEST(LineProjectionTests, Point2) {
    // Test case with 2 different points, where one of them is treated as a LineSegment.
    Vector2 start(10.0, -25.0), end(10.0, -25.0), point(0.0, 0.0);
    LineSegment lineSegment(start, end);
    Vector2 expected = {10.0, -25.0};
    Vector2 actual = lineSegment.project(point);
    EXPECT_FALSE(lineSegment.isOnLine(point));
    EXPECT_EQ(expected, actual);
}
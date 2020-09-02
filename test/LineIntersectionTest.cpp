//
// Created by Haico Dorenbos on 08-06-2020.
//
#include <gtest/gtest.h>
#include <math.h>
#include "roboteam_utils/HalfLine.h"
#include "roboteam_utils/Line.h"
#include "roboteam_utils/LineSegment.h"
#include "roboteam_utils/Vector2.h"

const double EPSILON = 1e-2;  // Small difference that is still significant enough to really change a Vector2 position.

using namespace rtt;

/* In this test class we test the different intersection methods in different line classes: Line, LineSegment and HalfLine. For these methods we want to cover 3 type of lines:
 * vertical line (does not change in x direction), horizontal line (does not change in y direction), arbitrary other line (does change in both x and y direction). This is to
 * test cases where there could be a possibility of division by zero. Furthermore for the non-parallel test cases we create 3 classes based on the relative position where they
 * intersect (or do not intersect): t < 0, 0 <= t <= 1, 1 < t. We extend these test cases with boundary value analysis, but for t is (minus) infinity we use a very small and
 * very large number (we expect intersections to fail for FLT_MAX and -FLT_MAX). Moreover for parallel test cases we create 3 classes based on the LineSegment interpretation:
 * 'Equal': both LineSegment interpretation are equal, 'Subset': one of the LineSegments interpretations is a subset of the other, 'Overlap': both LineSegments interpretations
 * have a shared part, but non of them is a subset of the other one, 'Distinct': both LineSegments do not intersect, but have a shared finite LineSegment where both are subsets
 * of, 'Different': both LineSegments do not intersect and do not have a shared infinite Line where both are subsets of. Furthermore create a test case for two LineSegments
 * that are 'Equal' points and that are 'Different' points.
 */
TEST(LineIntersectionTests, NonParallel1) {
    // Non-parallel test case with horizontal line and vertical line, where intersection happens for the first line at 0 < t < 1 and for the second at t = -EPSILON.
    Vector2 start1(3.0, 2.0), end1(6.0, 2.0), start2(4.0, 2 + EPSILON), end2(4.0, 5.0), intersection(4.0, 2.0);
    Line line1(start1, end1), line2(start2, end2);
    HalfLine halfLine(start1, end1);
    LineSegment lineSegment1(start1, end1), lineSegment2(start2, end2);
    std::optional<Vector2> result = halfLine.intersect(line2);
    EXPECT_EQ(result.value(), intersection);
    result = line1.intersect(line2);
    EXPECT_EQ(result.value(), intersection);
    result = Line::intersect(start1, end1, start2, end2);
    EXPECT_EQ(result.value(), intersection);
    result = lineSegment1.intersects(lineSegment2);
    EXPECT_FALSE(result.has_value());
    std::vector<Vector2> result2 = lineSegment1.multiIntersect(lineSegment2);
    EXPECT_EQ(result2.size(), 0);
}

TEST(LineIntersectionTests, NonParallel2) {
    // Non-parallel test case with two arbitrary lines, where intersection happens for the first line at t = 1 + EPSILON and for the second at 0 < t < 1.
    Vector2 start1(10.0, -6.0), end1(1.0 + EPSILON, -2 * EPSILON / 3), start2(0.0, -3.0), end2(4.0, 9.0), intersection(1.0, 0.0);
    Line line1(start1, end1), line2(start2, end2);
    HalfLine halfLine(start1, end1);
    LineSegment lineSegment1(start1, end1), lineSegment2(start2, end2);
    std::optional<Vector2> result = halfLine.intersect(line2);
    EXPECT_EQ(result.value(), intersection);
    result = line1.intersect(line2);
    EXPECT_EQ(result.value(), intersection);
    result = Line::intersect(start1, end1, start2, end2);
    EXPECT_EQ(result.value(), intersection);
    result = lineSegment1.intersects(lineSegment2);
    EXPECT_FALSE(result.has_value());
    std::vector<Vector2> result2 = lineSegment1.multiIntersect(lineSegment2);
    EXPECT_EQ(result2.size(), 0);
}

TEST(LineIntersectionTests, NonParallel3) {
    // Non-parallel test case with horizontal line and arbitrary line, where intersection happens for the first line at t = -EPSILON and for the second at t = LARGE.
    Vector2 start1(-2.0 + EPSILON, 1000.0), end1(8.0, 1000.0), start2(-502.0, -1000.0), end2(-500.0, -992.0), intersection(-2.0, 1000.0);
    Line line1(start1, end1), line2(start2, end2);
    HalfLine halfLine(start1, end1);
    LineSegment lineSegment1(start1, end1), lineSegment2(start2, end2);
    std::optional<Vector2> result = halfLine.intersect(line2);
    EXPECT_FALSE(result.has_value());
    result = line1.intersect(line2);
    EXPECT_EQ(result.value(), intersection);
    result = Line::intersect(start1, end1, start2, end2);
    EXPECT_EQ(result.value(), intersection);
    result = lineSegment1.intersects(lineSegment2);
    EXPECT_FALSE(result.has_value());
    std::vector<Vector2> result2 = lineSegment1.multiIntersect(lineSegment2);
    EXPECT_EQ(result2.size(), 0);
}

TEST(LineIntersectionTests, NonParallel4) {
    // Non-parallel test case with arbitrary line and vertical, where intersection happens for the first line at t = 1 and for the second at t = 0.
    Vector2 start1(-1.0, -4.0), end1(3.0, 8.0), start2(3.0, 8.0), end2(3.0, 6.5), intersection(3.0, 8.0);
    Line line1(start1, end1), line2(start2, end2);
    HalfLine halfLine(start1, end1);
    LineSegment lineSegment1(start1, end1), lineSegment2(start2, end2);
    std::optional<Vector2> result = halfLine.intersect(line2);
    EXPECT_EQ(result.value(), intersection);
    result = line1.intersect(line2);
    EXPECT_EQ(result.value(), intersection);
    result = Line::intersect(start1, end1, start2, end2);
    EXPECT_EQ(result.value(), intersection);
    result = lineSegment1.intersects(lineSegment2);
    EXPECT_EQ(result.value(), intersection);
    std::vector<Vector2> result2 = lineSegment1.multiIntersect(lineSegment2);
    EXPECT_EQ(result2.size(), 1);
    EXPECT_EQ(result2[0], intersection);
}

TEST(LineIntersectionTests, NonParallel5) {
    // Non-parallel test case with two arbitrary lines, where intersection happens for the first line at t = -LARGE and for the second at t = -LARGE.
    Vector2 start1(8.0, 49.8), end1(38.0, 50.0), start2(-22.0, -10.0), end2(8.0, -9.9), intersection(-17902.0, -69.6);
    Line line2(start2, end2);
    HalfLine halfLine(start1, end1);
    LineSegment lineSegment1(start1, end1), lineSegment2(start2, end2);
    std::optional<Vector2> result = halfLine.intersect(line2);
    EXPECT_FALSE(result.has_value());
    result = Line::intersect(start1, end1, start2, end2);
    EXPECT_EQ(result.value(), intersection);
    result = lineSegment1.intersects(lineSegment2);
    EXPECT_FALSE(result.has_value());
    std::vector<Vector2> result2 = lineSegment1.multiIntersect(lineSegment2);
    EXPECT_EQ(result2.size(), 0);
}

TEST(LineIntersectionTests, NonParallel6) {
    // Non-parallel test case a vertical and horizontal line, where intersection happens for the first line at t = LARGE and for the second at t = 1 + EPSILON.
    Vector2 start1(-10.0, 10000.0), end1(-10.0, 9999.9), start2(20.0, 5.0), end2(-10.0 + EPSILON, 5.0), intersection(-10.0, 5.0);
    Line line1(start1, end1), line2(start2, end2);
    HalfLine halfLine(start1, end1);
    LineSegment lineSegment1(start1, end1), lineSegment2(start2, end2);
    std::optional<Vector2> result = halfLine.intersect(line2);
    EXPECT_EQ(result.value(), intersection);
    result = line1.intersect(line2);
    EXPECT_EQ(result.value(), intersection);
    result = Line::intersect(start1, end1, start2, end2);
    EXPECT_EQ(result.value(), intersection);
    result = lineSegment1.intersects(lineSegment2);
    EXPECT_FALSE(result.has_value());
    std::vector<Vector2> result2 = lineSegment1.multiIntersect(lineSegment2);
    EXPECT_EQ(result2.size(), 0);
}

TEST(LineIntersectionTests, NonParallel7) {
    // Non-parallel test case a vertical line and arbitrary line, where intersection happens for the first line at t = 0 and for the second at 0 < t < 1.
    Vector2 start1(6.0, -2.0), end1(6.0, -500.0), start2(9.0, 1.0), end2(3.0, -5.0), intersection(6.0, -2.0);
    Line line1(start1, end1), line2(start2, end2);
    HalfLine halfLine(start1, end1);
    LineSegment lineSegment1(start1, end1), lineSegment2(start2, end2);
    std::optional<Vector2> result = halfLine.intersect(line2);
    EXPECT_EQ(result.value(), intersection);
    result = line1.intersect(line2);
    EXPECT_EQ(result.value(), intersection);
    result = Line::intersect(start1, end1, start2, end2);
    EXPECT_EQ(result.value(), intersection);
    result = lineSegment1.intersects(lineSegment2);
    EXPECT_EQ(result.value(), intersection);
    std::vector<Vector2> result2 = lineSegment1.multiIntersect(lineSegment2);
    EXPECT_EQ(result2.size(), 1);
    EXPECT_EQ(result2[0], intersection);
}

TEST(LineIntersectionTests, Parallel1) {
    // Parallel test case with two 'Equal' arbitrary lines.
    Vector2 start1(-1000.0, -800.0), end1(-200.0, -400.0), start2(-200.0, -400.0), end2(-1000.0, -800.0);
    Line line1(start1, end1), line2(start2, end2);
    HalfLine halfLine(start1, end1);
    LineSegment lineSegment1(start1, end1), lineSegment2(start2, end2);
    std::optional<Vector2> result = halfLine.intersect(line2);
    EXPECT_EQ(result.value(), Vector2(120.0, -240.0));
    result = line1.intersect(line2);
    EXPECT_EQ(result.value(), Vector2(120.0, -240.0));
    result = Line::intersect(start1, end1, start2, end2);
    EXPECT_FALSE(result.has_value());
    result = lineSegment1.intersects(lineSegment2);
    EXPECT_EQ(result.value(), start1);
    std::vector<Vector2> result2 = lineSegment1.multiIntersect(lineSegment2);
    EXPECT_EQ(result2.size(), 2);
    EXPECT_EQ(result2[0], end1);
    EXPECT_EQ(result2[1], start1);
}

TEST(LineIntersectionTests, Parallel2) {
    // Parallel test case where the first vertical line is a subset of the second vertical line.
    Vector2 start1(5.0, 3.0), end1(5.0, 6.0), start2(5.0, 12.0), end2(5.0, 3.0);
    Line line1(start1, end1), line2(start2, end2);
    HalfLine halfLine(start1, end1);
    LineSegment lineSegment1(start1, end1), lineSegment2(start2, end2);
    std::optional<Vector2> result = halfLine.intersect(line2);
    EXPECT_EQ(result.value(), start1);
    result = line1.intersect(line2);
    EXPECT_EQ(result.value(), Vector2(5.0, 0.0));
    result = Line::intersect(start1, end1, start2, end2);
    EXPECT_FALSE(result.has_value());
    result = lineSegment1.intersects(lineSegment2);
    EXPECT_EQ(result.value(), start1);
    std::vector<Vector2> result2 = lineSegment1.multiIntersect(lineSegment2);
    EXPECT_EQ(result2.size(), 2);
    EXPECT_EQ(result2[0], start1);
    EXPECT_EQ(result2[1], end1);
}

TEST(LineIntersectionTests, Parallel3) {
    // Parallel test case with two almost 'Distinct' horizontal lines
    Vector2 start1(-4.0, 10.0), end1(-3.0, 35.0), start2(-3.0 + EPSILON, 35.0 + 25 * EPSILON), end2(-1.0, 85.0);
    Line line1(start1, end1), line2(start2, end2);
    HalfLine halfLine(start1, end1);
    LineSegment lineSegment1(start1, end1), lineSegment2(start2, end2);
    std::optional<Vector2> result = halfLine.intersect(line2);
    EXPECT_EQ(result.value(), start1);
    result = line1.intersect(line2);
    EXPECT_EQ(result.value(), Vector2(-1375.0 / 313.0, 55.0 / 313.0));
    result = Line::intersect(start1, end1, start2, end2);
    EXPECT_FALSE(result.has_value());
    result = lineSegment1.intersects(lineSegment2);
    EXPECT_FALSE(result.has_value());
    std::vector<Vector2> result2 = lineSegment1.multiIntersect(lineSegment2);
    EXPECT_EQ(result2.size(), 0);
}

TEST(LineIntersectionTests, Parallel4) {
    // Parallel test case with two arbitrary 'Overlap' lines.
    Vector2 start1(-4.0, 10.0), end1(-3.0, 35.0), start2(-3.0, 35.0), end2(-1.0, 85.0);
    Line line1(start1, end1), line2(start2, end2);
    HalfLine halfLine(start1, end1);
    LineSegment lineSegment1(start1, end1), lineSegment2(start2, end2);
    std::optional<Vector2> result = halfLine.intersect(line2);
    EXPECT_EQ(result.value(), start1);
    result = line1.intersect(line2);
    EXPECT_EQ(result.value(), Vector2(-1375.0 / 313.0, 55.0 / 313.0));
    result = Line::intersect(start1, end1, start2, end2);
    EXPECT_FALSE(result.has_value());
    result = lineSegment1.intersects(lineSegment2);
    EXPECT_EQ(result.value(), end1);
    std::vector<Vector2> result2 = lineSegment1.multiIntersect(lineSegment2);
    EXPECT_EQ(result2.size(), 1);
    EXPECT_EQ(result2[0], end1);
}

TEST(LineIntersectionTests, Parallel5) {
    // Parallel test case with two 'Different' vertical lines.
    Vector2 start1(10.0, -5.0), end1(10.0, -15.0), start2(10.0 - EPSILON, -5.0), end2(10.0 - EPSILON, -15.0);
    Line line1(start1, end1), line2(start2, end2);
    HalfLine halfLine(start1, end1);
    LineSegment lineSegment1(start1, end1), lineSegment2(start2, end2);
    std::optional<Vector2> result = halfLine.intersect(line2);
    EXPECT_FALSE(result.has_value());
    result = line1.intersect(line2);
    EXPECT_FALSE(result.has_value());
    result = Line::intersect(start1, end1, start2, end2);
    EXPECT_FALSE(result.has_value());
    result = lineSegment1.intersects(lineSegment2);
    EXPECT_FALSE(result.has_value());
    std::vector<Vector2> result2 = lineSegment1.multiIntersect(lineSegment2);
    EXPECT_EQ(result2.size(), 0);
}

TEST(LineIntersectionTests, Point1) {
    // Parallel test case where the second line is actually a point that lies at the first horizontal line.
    Vector2 start1(15.0, -7.0), end1(3.0, -7.0), start2(11.0, -7.0), end2(11.0, -7.0);
    LineSegment lineSegment1(start1, end1), lineSegment2(start2, end2);
    std::optional<Vector2> result = Line::intersect(start1, end1, start2, end2);
    EXPECT_FALSE(result.has_value());
    result = lineSegment1.intersects(lineSegment2);
    EXPECT_EQ(result.value(), start2);
    std::vector<Vector2> result2 = lineSegment1.multiIntersect(lineSegment2);
    EXPECT_EQ(result2.size(), 1);
    EXPECT_EQ(result2[0], start2);
}

TEST(LineIntersectionTests, Point2) {
    // Test case with two lines that are actually 'Equal' points.
    Vector2 start1(0.0, 0.0), end1(0.0, 0.0), start2(0.0, 0.0), end2(0.0, 0.0);
    LineSegment lineSegment1(start1, end1), lineSegment2(start2, end2);
    std::optional<Vector2> result = Line::intersect(start1, end1, start2, end2);
    EXPECT_FALSE(result.has_value());
    result = lineSegment1.intersects(lineSegment2);
    EXPECT_EQ(result.value(), start1);
    std::vector<Vector2> result2 = lineSegment1.multiIntersect(lineSegment2);
    EXPECT_EQ(result2.size(), 1);
    EXPECT_EQ(result2[0], start1);
}

TEST(LineIntersectionTests, Point3) {
    // Test case with two lines that are actually 'Different' points.
    Vector2 start1(6.0 + EPSILON, 6.0), end1(6.0 + EPSILON, 6.0), start2(6.0, 6.0), end2(6.0, 6.0);
    LineSegment lineSegment1(start1, end1), lineSegment2(start2, end2);
    std::optional<Vector2> result = Line::intersect(start1, end1, start2, end2);
    EXPECT_FALSE(result.has_value());
    result = lineSegment1.intersects(lineSegment2);
    EXPECT_FALSE(result.has_value());
    std::vector<Vector2> result2 = lineSegment1.multiIntersect(lineSegment2);
    EXPECT_EQ(result2.size(), 0);
}

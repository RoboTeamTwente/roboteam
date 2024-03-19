#include <gtest/gtest.h>

#include <cmath>
#include <roboteam_utils/FastRectangle.hpp>

#include "roboteam_utils/Circle.h"
#include "roboteam_utils/Random.h"

using namespace rtt;

TEST(CircleTests, instantiation) {
    // Test unit circle
    Circle unitCircle;
    EXPECT_DOUBLE_EQ(unitCircle.center.x, 0);
    EXPECT_DOUBLE_EQ(unitCircle.center.y, 0);
    EXPECT_DOUBLE_EQ(unitCircle.radius, 1.0);
    // Normal instantiation
    Circle circle1({10, 20}, 2);
    EXPECT_DOUBLE_EQ(circle1.center.x, 10);
    EXPECT_DOUBLE_EQ(circle1.center.y, 20);
    EXPECT_DOUBLE_EQ(circle1.radius, 2.0);
    // Copy instantiation
    Circle circle2(circle1);
    EXPECT_DOUBLE_EQ(circle2.center.x, 10);
    EXPECT_DOUBLE_EQ(circle2.center.y, 20);
    EXPECT_DOUBLE_EQ(circle2.radius, 2.0);
    // Negative radius
    Circle circle3({30, 40}, -4);
    EXPECT_DOUBLE_EQ(circle3.center.x, 30);
    EXPECT_DOUBLE_EQ(circle3.center.y, 40);
    EXPECT_DOUBLE_EQ(circle3.radius, 4.0);
}

TEST(CircleTests, doesIntersectOrContainVector) {
    Circle circle({0, 0}, 1);
    // Inside the circle
    Vector2 vectorIn(0, 0);
    EXPECT_TRUE(circle.doesIntersectOrContain(vectorIn));
    // On the circle
    Vector2 vectorOn(1, 0);
    EXPECT_TRUE(circle.doesIntersectOrContain(vectorOn));
    // Outside the circle
    Vector2 vectorOut(10, 10);
    EXPECT_FALSE(circle.doesIntersectOrContain(vectorOut));
}

TEST(CircleTests, doesIntersectOrContainLine) {
    Circle circle({0, 0}, 1);
    // Inside the circle
    Line lineThrough({-1, 0}, {1, 0});
    EXPECT_TRUE(circle.doesIntersectOrContain(lineThrough));
    // On the circle
    Line lineOn({-1, 1}, {1, 1});
    EXPECT_TRUE(circle.doesIntersectOrContain(lineOn));
    // Outside the circle
    Line lineOut({-1, 2}, {1, 2});
    EXPECT_FALSE(circle.doesIntersectOrContain(lineOut));
}

TEST(CircleTests, doesIntersectOrContainLineSegment) {
    Circle circle({0, 0}, 1);
    // Inside the circle
    LineSegment lineThrough({-1, 0}, {1, 0});
    EXPECT_TRUE(circle.doesIntersectOrContain(lineThrough));
    // On the circle
    LineSegment lineOn({-1, 1}, {1, 1});
    EXPECT_TRUE(circle.doesIntersectOrContain(lineOn));
    // Outside the circle
    LineSegment lineOut({-1, 2}, {1, 2});
    EXPECT_FALSE(circle.doesIntersectOrContain(lineOut));
}

TEST(CircleTests, doesIntersectOrContainCircle) {
    Circle circle({0, 0}, 1);
    // Inside the circle
    Circle circleThrough({0, 0}, 1);
    EXPECT_TRUE(circle.doesIntersectOrContain(circleThrough));
    // On the circle
    Circle circleOn({2, 0}, 1);
    EXPECT_TRUE(circle.doesIntersectOrContain(circleOn));
    // Outside the circle
    Circle circleOut({4, 0}, 1);
    EXPECT_FALSE(circle.doesIntersectOrContain(circleOut));
}

TEST(CircleTests, doesIntersectOrContainRectangle) {
    Circle circle({0, 0}, 1);
    // Around the circle
    FastRectangle rectAround({2, 2}, {-2, -2});
    EXPECT_TRUE(circle.doesIntersectOrContain(rectAround));
    EXPECT_TRUE(circle.doesIntersectOrContain2(rectAround));
    // In the circle
    FastRectangle rectIn({.2, .2}, {-.2, -.2});
    EXPECT_TRUE(circle.doesIntersectOrContain(rectIn));
    EXPECT_TRUE(circle.doesIntersectOrContain2(rectIn));
    // Through the circle
    FastRectangle rectThrough({2, 2}, {0, 0});
    EXPECT_TRUE(circle.doesIntersectOrContain(rectThrough));
    EXPECT_TRUE(circle.doesIntersectOrContain2(rectThrough));
    // On the circle
    FastRectangle rectOn({1, 1}, {2, 0});
    EXPECT_TRUE(circle.doesIntersectOrContain(rectOn));
    EXPECT_TRUE(circle.doesIntersectOrContain2(rectOn));
    // Outside the circle
    FastRectangle rectOutside({4, 4}, {2, 2});
    EXPECT_FALSE(circle.doesIntersectOrContain(rectOutside));
    EXPECT_FALSE(circle.doesIntersectOrContain2(rectOutside));
}

TEST(CircleTests, doesIntersectOrContainVector2Test) {
    Circle circle({0, 0}, 1);
    Vector2 pointInside(0, 0);
    Vector2 pointOnEdge(1, 0);
    Vector2 pointOutside(2, 0);
    EXPECT_TRUE(circle.doesIntersectOrContain(pointInside));
    EXPECT_TRUE(circle.doesIntersectOrContain(pointOnEdge));
    EXPECT_FALSE(circle.doesIntersectOrContain(pointOutside));
}

TEST(CircleTests, projectTest) {
    Circle circle({0, 0}, 1);
    // Inside the circle
    Vector2 vectorIn(0.5, 0);
    Vector2 vectorInP = circle.project(vectorIn);
    EXPECT_DOUBLE_EQ(vectorInP.x, 1);
    EXPECT_DOUBLE_EQ(vectorInP.y, 0);
    // On the circle
    Vector2 vectorOn(1, 0);
    Vector2 vectorOnP = circle.project(vectorOn);
    EXPECT_DOUBLE_EQ(vectorOnP.x, 1);
    EXPECT_DOUBLE_EQ(vectorOnP.y, 0);
    // Outside the circle
    Vector2 vectorOut(2, 0);
    Vector2 vectorOutP = circle.project(vectorOut);
    EXPECT_DOUBLE_EQ(vectorOutP.x, 1);
    EXPECT_DOUBLE_EQ(vectorOutP.y, 0);
    // Exactly on the circle
    Vector2 vectorZero(0, 0);
    Vector2 vectorZeroP = circle.project(vectorZero);
    EXPECT_DOUBLE_EQ(vectorZeroP.x, 1);
    EXPECT_DOUBLE_EQ(vectorZeroP.y, 0);
    // Under a 45 degree angle
    Vector2 vectorAngle(2, 2);
    Vector2 vectorAngleP = circle.project(vectorAngle);
    EXPECT_DOUBLE_EQ(vectorAngleP.x, std::sqrt(2) / 2);
    EXPECT_DOUBLE_EQ(vectorAngleP.y, std::sqrt(2) / 2);
}

TEST(CircleTests, operatorPlus) {
    Circle circle({0, 0}, 1);
    Vector2 vector({1, 1});
    Circle circle2 = circle + vector;
    EXPECT_DOUBLE_EQ(circle2.center.x, 1);
    EXPECT_DOUBLE_EQ(circle2.center.y, 1);
    circle2 += circle2.center;
    EXPECT_DOUBLE_EQ(circle2.center.x, 2);
    EXPECT_DOUBLE_EQ(circle2.center.y, 2);
}

TEST(CircleTests, operatorMinus) {
    Circle circle({0, 0}, 1);
    Vector2 vector({1, 1});
    Circle circle2 = circle - vector;
    EXPECT_DOUBLE_EQ(circle2.center.x, -1);
    EXPECT_DOUBLE_EQ(circle2.center.y, -1);
    circle2 -= circle2.center;
    EXPECT_DOUBLE_EQ(circle2.center.x, 0);
    EXPECT_DOUBLE_EQ(circle2.center.y, 0);
}

TEST(CircleTests, operatorMultiply) {
    Circle circle({1, 1}, 1);
    Circle circle2 = circle * 3;
    EXPECT_DOUBLE_EQ(circle2.radius, 3);
    circle2 *= 3;
    EXPECT_DOUBLE_EQ(circle2.radius, 9);
}

TEST(CircleTests, operatorDivide) {
    Circle circle({1, 1}, 1);
    Circle circle2 = circle / 3;
    EXPECT_DOUBLE_EQ(circle2.radius, 1. / 3);
    circle2 /= 3;
    EXPECT_DOUBLE_EQ(circle2.radius, 1. / 9);
    circle2 /= 0;
    EXPECT_TRUE(std::isinf(circle2.radius));
}

TEST(CircleTests, intersectionsTest) {
    Circle circle({0, 0}, 1);
    LineSegment noIntersectLine({-2, 2}, {2, 2});
    ASSERT_TRUE(circle.intersects(noIntersectLine).empty());
    LineSegment noIntersectLineSegment({2, 0}, {3, 0});
    ASSERT_TRUE(circle.intersects(noIntersectLineSegment).empty());

    LineSegment oneIntersectLine({-2, 1}, {2, 1});
    ASSERT_EQ(circle.intersects(oneIntersectLine).size(), 1);
    ASSERT_EQ(circle.intersects(oneIntersectLine).at(0), Vector2(0, 1));

    LineSegment twoIntersectLine({-2, 0}, {2, 0});
    ASSERT_EQ(circle.intersects(twoIntersectLine).size(), 2);
    auto intersections = circle.intersects(twoIntersectLine);
    auto possibility1 = std::vector<Vector2>{Vector2(1, 0), Vector2(-1, 0)};
    auto possibility2 = std::vector<Vector2>{Vector2(-1, 0), Vector2(1, 0)};
    ASSERT_TRUE(intersections == possibility1 || intersections == possibility2);

    LineSegment containsOneEnd({-2, 0}, {0, 0});
    ASSERT_EQ(circle.intersects(containsOneEnd).size(), 1);
    ASSERT_EQ(circle.intersects(containsOneEnd).at(0), Vector2(-1, 0));

    LineSegment containsTwoEnds({-0.5, 0}, {0.5, 0});
    for (const auto& a : circle.intersects(containsTwoEnds)) {
        std::cout << a << std::endl;
    }
    ASSERT_TRUE(circle.intersects(containsTwoEnds).empty());

    LineSegment diagonalLineSegment({-2, -2}, {2, 2});
    auto intersectionsDiagonal = circle.intersects(diagonalLineSegment);
    auto possibilityDiagonal1 = std::vector<Vector2>{Vector2(-sqrt(2) / 2, -sqrt(2) / 2), Vector2(sqrt(2) / 2, sqrt(2) / 2)};
    auto possibilityDiagonal2 = std::vector<Vector2>{Vector2(sqrt(2) / 2, sqrt(2) / 2), Vector2(-sqrt(2) / 2, -sqrt(2) / 2)};
    ASSERT_TRUE(intersectionsDiagonal == possibilityDiagonal1 || intersectionsDiagonal == possibilityDiagonal2);
}

TEST(CircleTests, moveTest) {
    Circle circle({0, 0}, 1);
    // Move by a positive vector
    Vector2 vector1(1, 1);
    circle.move(vector1);
    EXPECT_DOUBLE_EQ(circle.center.x, 1);
    EXPECT_DOUBLE_EQ(circle.center.y, 1);
    // Move by a negative vector
    Vector2 vector2(-2, -2);
    circle.move(vector2);
    EXPECT_DOUBLE_EQ(circle.center.x, -1);
    EXPECT_DOUBLE_EQ(circle.center.y, -1);
}
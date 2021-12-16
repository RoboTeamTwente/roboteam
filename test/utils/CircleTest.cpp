//
// Created by emiel on 25-02-20.
//

#include <gtest/gtest.h>

#include <cmath>

#include "roboteam_utils/Circle.h"

TEST(CircleTests, instantiation) {
    // Test unit circle
    rtt::Circle unitCircle;
    EXPECT_DOUBLE_EQ(unitCircle.center.x, 0);
    EXPECT_DOUBLE_EQ(unitCircle.center.y, 0);
    EXPECT_DOUBLE_EQ(unitCircle.radius, 1.0);
    // Normal instantiation
    rtt::Circle circle1({10, 20}, 2);
    EXPECT_DOUBLE_EQ(circle1.center.x, 10);
    EXPECT_DOUBLE_EQ(circle1.center.y, 20);
    EXPECT_DOUBLE_EQ(circle1.radius, 2.0);
    // Copy instantiation
    rtt::Circle circle2(circle1);
    EXPECT_DOUBLE_EQ(circle2.center.x, 10);
    EXPECT_DOUBLE_EQ(circle2.center.y, 20);
    EXPECT_DOUBLE_EQ(circle2.radius, 2.0);
    // Negative radius
    rtt::Circle circle3({30, 40}, -4);
    EXPECT_DOUBLE_EQ(circle3.center.x, 30);
    EXPECT_DOUBLE_EQ(circle3.center.y, 40);
    EXPECT_DOUBLE_EQ(circle3.radius, 4.0);
}

TEST(CircleTests, doesIntersectOrContainVector) {
    rtt::Circle circle({0, 0}, 1);
    // Inside the circle
    rtt::Vector2 vectorIn(0, 0);
    EXPECT_TRUE(circle.doesIntersectOrContain(vectorIn));
    // On the circle
    rtt::Vector2 vectorOn(1, 0);
    EXPECT_TRUE(circle.doesIntersectOrContain(vectorOn));
    // Outside the circle
    rtt::Vector2 vectorOut(10, 10);
    EXPECT_FALSE(circle.doesIntersectOrContain(vectorOut));
}

TEST(CircleTests, doesIntersectOrContainLine) {
    rtt::Circle circle({0, 0}, 1);
    // Inside the circle
    rtt::Line lineThrough({-1, 0}, {1, 0});
    EXPECT_TRUE(circle.doesIntersectOrContain(lineThrough));
    // On the circle
    rtt::Line lineOn({-1, 1}, {1, 1});
    EXPECT_TRUE(circle.doesIntersectOrContain(lineOn));
    // Outside the circle
    rtt::Line lineOut({-1, 2}, {1, 2});
    EXPECT_FALSE(circle.doesIntersectOrContain(lineOut));
}

TEST(CircleTests, doesIntersectOrContainLineSegment) {
    rtt::Circle circle({0, 0}, 1);
    // Inside the circle
    rtt::LineSegment lineThrough({-1, 0}, {1, 0});
    EXPECT_TRUE(circle.doesIntersectOrContain(lineThrough));
    // On the circle
    rtt::LineSegment lineOn({-1, 1}, {1, 1});
    EXPECT_TRUE(circle.doesIntersectOrContain(lineOn));
    // Outside the circle
    rtt::LineSegment lineOut({-1, 2}, {1, 2});
    EXPECT_FALSE(circle.doesIntersectOrContain(lineOut));
}

TEST(CircleTests, doesIntersectOrContainCircle) {
    rtt::Circle circle({0, 0}, 1);
    // Inside the circle
    rtt::Circle circleThrough({0, 0}, 1);
    EXPECT_TRUE(circle.doesIntersectOrContain(circleThrough));
    // On the circle
    rtt::Circle circleOn({2, 0}, 1);
    EXPECT_TRUE(circle.doesIntersectOrContain(circleOn));
    // Outside the circle
    rtt::Circle circleOut({4, 0}, 1);
    EXPECT_FALSE(circle.doesIntersectOrContain(circleOut));
}

TEST(CircleTests, doesIntersectOrContainRectangle) {
    rtt::Circle circle({0, 0}, 1);
    // Around the circle
    rtt::Rectangle rectAround({2, 2}, {-2, -2});
    EXPECT_TRUE(circle.doesIntersectOrContain(rectAround));
    EXPECT_TRUE(circle.doesIntersectOrContain2(rectAround));
    // In the circle
    rtt::Rectangle rectIn({.2, .2}, {-.2, -.2});
    EXPECT_TRUE(circle.doesIntersectOrContain(rectIn));
    EXPECT_TRUE(circle.doesIntersectOrContain2(rectIn));
    // Through the circle
    rtt::Rectangle rectThrough({2, 2}, {0, 0});
    EXPECT_TRUE(circle.doesIntersectOrContain(rectThrough));
    EXPECT_TRUE(circle.doesIntersectOrContain2(rectThrough));
    // On the circle
    rtt::Rectangle rectOn({1, 1}, {2, 0});
    EXPECT_TRUE(circle.doesIntersectOrContain(rectOn));
    EXPECT_TRUE(circle.doesIntersectOrContain2(rectOn));
    // Outside the circle
    rtt::Rectangle rectOutside({4, 4}, {2, 2});
    EXPECT_FALSE(circle.doesIntersectOrContain(rectOutside));
    EXPECT_FALSE(circle.doesIntersectOrContain2(rectOutside));
}

TEST(CircleTests, projectTest) {
    rtt::Circle circle({0, 0}, 1);
    // Inside the circle
    rtt::Vector2 vectorIn(0.5, 0);
    rtt::Vector2 vectorInP = circle.project(vectorIn);
    EXPECT_DOUBLE_EQ(vectorInP.x, 1);
    EXPECT_DOUBLE_EQ(vectorInP.y, 0);
    // On the circle
    rtt::Vector2 vectorOn(1, 0);
    rtt::Vector2 vectorOnP = circle.project(vectorOn);
    EXPECT_DOUBLE_EQ(vectorOnP.x, 1);
    EXPECT_DOUBLE_EQ(vectorOnP.y, 0);
    // Outside the circle
    rtt::Vector2 vectorOut(2, 0);
    rtt::Vector2 vectorOutP = circle.project(vectorOut);
    EXPECT_DOUBLE_EQ(vectorOutP.x, 1);
    EXPECT_DOUBLE_EQ(vectorOutP.y, 0);
    // Exactly on the circle
    rtt::Vector2 vectorZero(0, 0);
    rtt::Vector2 vectorZeroP = circle.project(vectorZero);
    EXPECT_DOUBLE_EQ(vectorZeroP.x, 1);
    EXPECT_DOUBLE_EQ(vectorZeroP.y, 0);
    // Under a 45 degree angle
    rtt::Vector2 vectorAngle(2, 2);
    rtt::Vector2 vectorAngleP = circle.project(vectorAngle);
    EXPECT_DOUBLE_EQ(vectorAngleP.x, std::sqrt(2) / 2);
    EXPECT_DOUBLE_EQ(vectorAngleP.y, std::sqrt(2) / 2);
}

TEST(CircleTests, operatorPlus) {
    rtt::Circle circle({0, 0}, 1);
    rtt::Vector2 vector({1, 1});
    rtt::Circle circle2 = circle + vector;
    EXPECT_DOUBLE_EQ(circle2.center.x, 1);
    EXPECT_DOUBLE_EQ(circle2.center.y, 1);
    circle2 += circle2.center;
    EXPECT_DOUBLE_EQ(circle2.center.x, 2);
    EXPECT_DOUBLE_EQ(circle2.center.y, 2);
}

TEST(CircleTests, operatorMinus) {
    rtt::Circle circle({0, 0}, 1);
    rtt::Vector2 vector({1, 1});
    rtt::Circle circle2 = circle - vector;
    EXPECT_DOUBLE_EQ(circle2.center.x, -1);
    EXPECT_DOUBLE_EQ(circle2.center.y, -1);
    circle2 -= circle2.center;
    EXPECT_DOUBLE_EQ(circle2.center.x, 0);
    EXPECT_DOUBLE_EQ(circle2.center.y, 0);
}

TEST(CircleTests, operatorMultiply) {
    rtt::Circle circle({1, 1}, 1);
    rtt::Circle circle2 = circle * 3;
    EXPECT_DOUBLE_EQ(circle2.radius, 3);
    circle2 *= 3;
    EXPECT_DOUBLE_EQ(circle2.radius, 9);
}

TEST(CircleTests, operatorDivide) {
    rtt::Circle circle({1, 1}, 1);
    rtt::Circle circle2 = circle / 3;
    EXPECT_DOUBLE_EQ(circle2.radius, 1. / 3);
    circle2 /= 3;
    EXPECT_DOUBLE_EQ(circle2.radius, 1. / 9);
    circle2 /= 0;
    EXPECT_TRUE(std::isinf(circle2.radius));
}

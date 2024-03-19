#include <gtest/gtest.h>
#include <roboteam_utils/Random.h>

#include <roboteam_utils/Ball.hpp>

using namespace rtt;

TEST(BallTests, instantiation) {
    Ball ball1;
    ASSERT_EQ(ball1.position, Vector2(0, 0));
    ASSERT_DOUBLE_EQ(ball1.height, 0.0);
    ASSERT_EQ(ball1.velocity, Vector2(0, 0));
    ASSERT_DOUBLE_EQ(ball1.verticalVelocity, 0.0);
    ASSERT_EQ(ball1.expectedEndPosition, Vector2(0, 0));
    ASSERT_FALSE(ball1.isVisible);
    ASSERT_EQ(ball1.area, 0);

    // Test instantiation with non-default values
    Ball ball2 = {.position = Vector2(1.0, 2.0),
                  .height = 3.0,
                  .velocity = Vector2(4.0, 5.0),
                  .verticalVelocity = 6.0,
                  .expectedEndPosition = Vector2(7.0, 8.0),
                  .isVisible = true,
                  .area = 9};

    ASSERT_EQ(ball2.position, Vector2(1.0, 2.0));
    ASSERT_DOUBLE_EQ(ball2.height, 3.0);
    ASSERT_EQ(ball2.velocity, Vector2(4.0, 5.0));
    ASSERT_DOUBLE_EQ(ball2.verticalVelocity, 6.0);
    ASSERT_EQ(ball2.expectedEndPosition, Vector2(7.0, 8.0));
    ASSERT_TRUE(ball2.isVisible);
    ASSERT_EQ(ball2.area, 9);
}

TEST(BallTests, equality) {
    Ball ball = {.position = Vector2(SimpleRandom::getDouble(-10.0, 10.0), SimpleRandom::getDouble(-10.0, 10.0)),
                 .height = SimpleRandom::getDouble(0.0, 10.0),
                 .velocity = Vector2(SimpleRandom::getDouble(-10.0, 10.0), SimpleRandom::getDouble(-10.0, 10.0)),
                 .verticalVelocity = SimpleRandom::getDouble(-10.0, 10.0),
                 .expectedEndPosition = Vector2(SimpleRandom::getDouble(-10.0, 10.0), SimpleRandom::getDouble(-10.0, 10.0)),
                 .isVisible = SimpleRandom::getBool(),
                 .area = static_cast<unsigned int>(SimpleRandom::getInt(0, 10))};

    Ball copy = ball;
    ASSERT_EQ(ball, copy);

    // Test inequality
    Ball ball1 = {.position = Vector2(1.0, 2.0),
                  .height = 3.0,
                  .velocity = Vector2(4.0, 5.0),
                  .verticalVelocity = 6.0,
                  .expectedEndPosition = Vector2(7.0, 8.0),
                  .isVisible = true,
                  .area = 9};

    Ball ball2 = {.position = Vector2(10.0, 20.0),
                  .height = 30.0,
                  .velocity = Vector2(40.0, 50.0),
                  .verticalVelocity = 60.0,
                  .expectedEndPosition = Vector2(70.0, 80.0),
                  .isVisible = false,
                  .area = 90};

    ASSERT_NE(ball1, ball2);
}
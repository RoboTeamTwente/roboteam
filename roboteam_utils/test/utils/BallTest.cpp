#include <gtest/gtest.h>
#include <roboteam_utils/Random.h>

#include <roboteam_utils/Ball.hpp>

using namespace rtt;

TEST(BallTests, instantiation) {
    Ball ball;
    ASSERT_EQ(ball.position, Vector2(0, 0));
    ASSERT_DOUBLE_EQ(ball.height, 0.0);
    ASSERT_EQ(ball.velocity, Vector2(0, 0));
    ASSERT_DOUBLE_EQ(ball.verticalVelocity, 0.0);
    ASSERT_EQ(ball.expectedEndPosition, Vector2(0, 0));
    ASSERT_FALSE(ball.isVisible);
    ASSERT_EQ(ball.area, 0);
}

TEST(BallTests, equality) {
    Ball ball = {
        .position = Vector2(SimpleRandom::getDouble(-10.0, 10.0), SimpleRandom::getDouble(-10.0, 10.0)),
        .height = SimpleRandom::getDouble(0.0, 10.0),
        .velocity = Vector2(SimpleRandom::getDouble(-10.0, 10.0), SimpleRandom::getDouble(-10.0, 10.0)),
        .verticalVelocity = SimpleRandom::getDouble(-10.0, 10.0),
        .expectedEndPosition = Vector2(SimpleRandom::getDouble(-10.0, 10.0), SimpleRandom::getDouble(-10.0, 10.0)),
        .isVisible = SimpleRandom::getBool(),
        .area = static_cast<unsigned int>(SimpleRandom::getInt(0, 10))
    };

    Ball copy = ball;
    ASSERT_EQ(ball, copy);
}
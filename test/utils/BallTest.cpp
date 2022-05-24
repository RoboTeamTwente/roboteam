#include <gtest/gtest.h>
#include <roboteam_utils/Ball.hpp>

#include <random>

using namespace rtt;

double randomD(double low, double high) {
    std::uniform_real_distribution<double> distribution(low, high);
    std::default_random_engine engine;
    return distribution(engine);
}

bool randomB() {
    std::uniform_int_distribution distribution(0, 1);
    std::default_random_engine engine;
    return distribution(engine);
}

int randomI(int low, int high) {
    std::uniform_int_distribution distribution(low, high);
    std::default_random_engine engine;
    return distribution(engine);
}

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
        .position = Vector2(randomD(-10.0, 10.0), randomD(-10.0, 10.0)),
        .height = randomD(0.0, 10.0),
        .velocity = Vector2(randomD(-10.0, 10.0), randomD(-10.0, 10.0)),
        .verticalVelocity = randomD(-10.0, 10.0),
        .expectedEndPosition = Vector2(randomD(-10.0, 10.0), randomD(-10.0, 10.0)),
        .isVisible = randomB(),
        .area = static_cast<unsigned int>(randomI(0, 10))
    };

    Ball copy = ball;
    ASSERT_EQ(ball, copy);
}
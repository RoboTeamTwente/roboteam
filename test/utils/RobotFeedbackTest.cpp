#include <gtest/gtest.h>
#include <roboteam_utils/RobotFeedback.hpp>

#include <random>

using namespace rtt;

float randomFloat(float low, float high) {
    std::uniform_real_distribution<float> distribution(low, high);
    std::default_random_engine engine;
    return distribution(engine);
}
int randomInt(int low, int high) {
    std::uniform_int_distribution distribution(low, high);
    std::default_random_engine engine;
    return distribution(engine);
}
bool randomBool() {
    std::uniform_int_distribution distribution(0, 1);
    std::default_random_engine engine;
    return distribution(engine);
}
RobotFeedbackSource randomFeedbackSource() {
    return randomBool() ? RobotFeedbackSource::SIMULATOR : RobotFeedbackSource::BASESTATION;
}
Team randomTeam() {
    return randomBool() ? Team::YELLOW : Team::BLUE;
}

RobotFeedback randomFeedback() {
    RobotFeedback feedback = {
        .id = randomInt(0, 15),
        .hasBall = randomBool(),
        .ballPosition = randomFloat(-0.5, 0.5),
        .ballSensorIsWorking = randomBool(),
        .velocity = Vector2(randomFloat(-10.0, 10.0), randomFloat(-10.0, 10.0)),
        .angle = Angle(randomFloat(-M_PI, M_PI)),
        .xSensIsCalibrated = randomBool(),
        .capacitorIsCharged = randomBool(),
        .wheelLocked = randomInt(0, 15),
        .wheelBraking = randomInt(0, 15),
        .batteryLevel = randomFloat(18.0f, 26.0f),
        .signalStrength = randomInt(0, 255)
    };
    return feedback;
}

// Make sure an empty robot command is initialized correctly
TEST(RobotFeedbackTest, instantiation) {
    RobotFeedback robotFeedback;
    ASSERT_EQ(robotFeedback.id, 0);
    ASSERT_FALSE(robotFeedback.hasBall);
    ASSERT_FLOAT_EQ(robotFeedback.ballPosition, 0.0f);
    ASSERT_FALSE(robotFeedback.ballSensorIsWorking);
    ASSERT_EQ(robotFeedback.velocity, Vector2());
    ASSERT_EQ(robotFeedback.angle, Angle());
    ASSERT_FALSE(robotFeedback.xSensIsCalibrated);
    ASSERT_FALSE(robotFeedback.capacitorIsCharged);
    ASSERT_EQ(robotFeedback.wheelLocked, 0);
    ASSERT_EQ(robotFeedback.wheelBraking, 0);
    ASSERT_FLOAT_EQ(robotFeedback.batteryLevel, 23.0f);
    ASSERT_EQ(robotFeedback.signalStrength, 0);
}

TEST(RobotFeedbackTest, equals) {
    RobotFeedback random = randomFeedback();
    RobotFeedback copy = random;
    ASSERT_EQ(random, copy);
}

TEST(RobotsFeedbackTest, instantiate) {
    RobotsFeedback feedbacks;
    ASSERT_EQ(feedbacks.team, Team::YELLOW);
    ASSERT_EQ(feedbacks.source, RobotFeedbackSource::SIMULATOR);
    ASSERT_EQ(feedbacks.feedback.size(), 0);
}

TEST(RobotsFeedbackTest, equals) {
    RobotsFeedback feedbacks = {
        .team = randomTeam(),
        .source = randomFeedbackSource(),
        .feedback = { randomFeedback(), randomFeedback(), randomFeedback() }
    };

    RobotsFeedback copy = feedbacks;
    ASSERT_EQ(feedbacks, copy);
}
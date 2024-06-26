#include <gtest/gtest.h>
#include <roboteam_utils/Random.h>

#include <roboteam_utils/RobotFeedback.hpp>

using namespace rtt;

RobotFeedbackSource randomFeedbackSource() {
    std::vector<RobotFeedbackSource> allSources = {RobotFeedbackSource::SIMULATOR, RobotFeedbackSource::BASESTATION};
    return *SimpleRandom::getRandomElement(allSources.begin(), allSources.end());
}
Team randomTeam() {
    std::vector<Team> allTeams = {Team::YELLOW, Team::BLUE};
    return *SimpleRandom::getRandomElement(allTeams.begin(), allTeams.end());
}

RobotFeedback randomFeedback() {
    RobotFeedback feedback = {.id = SimpleRandom::getInt(0, 15),
                              .ballSensorSeesBall = SimpleRandom::getBool(),
                              .ballSensorIsWorking = SimpleRandom::getBool(),
                              .dribblerSeesBall = SimpleRandom::getBool(),
                              .velocity = Vector2(SimpleRandom::getDouble(-10.0, 10.0), SimpleRandom::getDouble(-10.0, 10.0)),
                              .yaw = Angle(SimpleRandom::getDouble(-M_PI, M_PI)),
                              .xSensIsCalibrated = SimpleRandom::getBool(),
                              .capacitorIsCharged = SimpleRandom::getBool(),
                              .batteryLevel = static_cast<float>(SimpleRandom::getDouble(18.0f, 26.0f))};
    return feedback;
}

// Make sure an empty robot command is initialized correctly
TEST(RobotFeedbackTest, instantiation) {
    RobotFeedback robotFeedback;
    ASSERT_EQ(robotFeedback.id, 0);
    ASSERT_FALSE(robotFeedback.ballSensorSeesBall);
    ASSERT_FALSE(robotFeedback.ballSensorIsWorking);
    ASSERT_FALSE(robotFeedback.dribblerSeesBall);
    ASSERT_EQ(robotFeedback.velocity, Vector2());
    ASSERT_EQ(robotFeedback.yaw, Angle());
    ASSERT_FALSE(robotFeedback.xSensIsCalibrated);
    ASSERT_FALSE(robotFeedback.capacitorIsCharged);
    ASSERT_FLOAT_EQ(robotFeedback.batteryLevel, 23.0f);
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
    RobotsFeedback feedbacks = {.team = randomTeam(), .source = randomFeedbackSource(), .feedback = {randomFeedback(), randomFeedback(), randomFeedback()}};

    RobotsFeedback copy = feedbacks;
    ASSERT_EQ(feedbacks, copy);
}
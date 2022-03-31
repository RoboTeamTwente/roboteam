#include <gtest/gtest.h>
#include <roboteam_utils/RobotCommands.hpp>

#include <random>

using namespace rtt;

double randomD(double low, double high) {
    std::uniform_real_distribution<double> distribution(low, high);
    std::default_random_engine engine;
    return distribution(engine);
}
int randomI(int low, int high) {
    std::uniform_int_distribution distribution(low, high);
    std::default_random_engine engine;
    return distribution(engine);
}
bool randomB() {
    std::uniform_int_distribution distribution(0, 1);
    std::default_random_engine engine;
    return distribution(engine);
}
KickType randomKickType() {
    int i = randomI(0, 2);
    switch (i) {
        case 0:
            return KickType::NO_KICK;
        case 1:
            return KickType::KICK;
        case 2:
            return KickType::CHIP;
        default:
            return KickType::NO_KICK;
    }
}

// Make sure an empty robot command is initialized correctly
TEST(RobotCommandsTest, instantiation) {
    RobotCommand robotCommand;
    ASSERT_EQ(robotCommand.id, 0);
    ASSERT_EQ(robotCommand.velocity, Vector2());
    ASSERT_EQ(robotCommand.targetAngle, Angle());
    ASSERT_DOUBLE_EQ(robotCommand.targetAngularVelocity, 0.0);
    ASSERT_FALSE(robotCommand.useAngularVelocity);

    ASSERT_EQ(robotCommand.cameraAngleOfRobot, Angle());
    ASSERT_FALSE(robotCommand.cameraAngleOfRobotIsSet);
    ASSERT_DOUBLE_EQ(robotCommand.kickSpeed, 0.0);
    ASSERT_FALSE(robotCommand.waitForBall);
    ASSERT_EQ(robotCommand.kickType, KickType::NO_KICK);
    ASSERT_DOUBLE_EQ(robotCommand.dribblerSpeed, 0.0);

    ASSERT_FALSE(robotCommand.ignorePacket);
}

TEST(RobotCommandsTest, equals) {
    RobotCommand command = {
        .id = randomI(0, 15),
        .velocity = Vector2(randomD(-10.0, 10.0), randomD(-10.0, 10.0)),
        .targetAngle = Angle(randomD(-M_PI, M_PI)),
        .targetAngularVelocity = randomD(-5.0, 5.0),
        .useAngularVelocity = randomB(),
        .cameraAngleOfRobot = Angle(randomD(-M_PI, M_PI)),
        .cameraAngleOfRobotIsSet = randomB(),
        .kickSpeed = randomD(0.0, 5.0),
        .waitForBall = randomB(),
        .kickType = randomKickType(),
        .dribblerSpeed = randomD(0.0, 5.0),
        .ignorePacket = randomB()
    };
    RobotCommand copy = command;
    ASSERT_EQ(command, copy);
}
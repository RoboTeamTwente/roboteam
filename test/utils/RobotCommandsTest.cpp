#include <gtest/gtest.h>
#include <roboteam_utils/RobotCommands.hpp>

#include <roboteam_utils/Random.h>

using namespace rtt;

KickType randomKickType() {
    std::vector<KickType> allKicks = { KickType::NO_KICK, KickType::KICK, KickType::CHIP };
    return *SimpleRandom::getRandomElement(allKicks.begin(), allKicks.end());
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
    ASSERT_FALSE(robotCommand.kickAtAngle);
    ASSERT_DOUBLE_EQ(robotCommand.dribblerSpeed, 0.0);

    ASSERT_FALSE(robotCommand.ignorePacket);
}

TEST(RobotCommandsTest, equals) {
    RobotCommand command = {
        .id = SimpleRandom::getInt(0, 15),
        .velocity = Vector2(SimpleRandom::getDouble(-10.0, 10.0), SimpleRandom::getDouble(-10.0, 10.0)),
        .targetAngle = Angle(SimpleRandom::getDouble(-M_PI, M_PI)),
        .targetAngularVelocity = SimpleRandom::getDouble(-5.0, 5.0),
        .useAngularVelocity = SimpleRandom::getBool(),
        .cameraAngleOfRobot = Angle(SimpleRandom::getDouble(-M_PI, M_PI)),
        .cameraAngleOfRobotIsSet = SimpleRandom::getBool(),
        .kickSpeed = SimpleRandom::getDouble(0.0, 5.0),
        .waitForBall = SimpleRandom::getBool(),
        .kickType = randomKickType(),
        .kickAtAngle = SimpleRandom::getBool(),
        .dribblerSpeed = SimpleRandom::getDouble(0.0, 5.0),
        .ignorePacket = SimpleRandom::getBool()
    };
    RobotCommand copy = command;
    ASSERT_EQ(command, copy);
}
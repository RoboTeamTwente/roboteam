#include <gtest/gtest.h>
#include <roboteam_utils/Random.h>

#include <roboteam_utils/RobotCommands.hpp>

using namespace rtt;

KickType randomKickType() {
    std::vector<KickType> allKicks = {KickType::NO_KICK, KickType::KICK, KickType::CHIP};
    return *SimpleRandom::getRandomElement(allKicks.begin(), allKicks.end());
}

// Make sure an empty robot command is initialized correctly
TEST(RobotCommandsTest, instantiation) {
    RobotCommand robotCommand;
    ASSERT_EQ(robotCommand.id, 0);
    ASSERT_EQ(robotCommand.velocity, Vector2());
    ASSERT_EQ(robotCommand.yaw, Angle());
    ASSERT_DOUBLE_EQ(robotCommand.targetAngularVelocity, 0.0);
    ASSERT_FALSE(robotCommand.useAngularVelocity);

    ASSERT_EQ(robotCommand.cameraYawOfRobot, Angle());
    ASSERT_FALSE(robotCommand.cameraYawOfRobotIsSet);
    ASSERT_DOUBLE_EQ(robotCommand.kickSpeed, 0.0);
    ASSERT_FALSE(robotCommand.waitForBall);
    ASSERT_EQ(robotCommand.kickType, KickType::NO_KICK);
    ASSERT_FALSE(robotCommand.kickAtYaw);
    ASSERT_DOUBLE_EQ(robotCommand.dribblerOn, 0.0);

    ASSERT_FALSE(robotCommand.wheelsOff);
}

TEST(RobotCommandsTest, equals) {
    RobotCommand command = {.id = SimpleRandom::getInt(0, 15),
                            .velocity = Vector2(SimpleRandom::getDouble(-10.0, 10.0), SimpleRandom::getDouble(-10.0, 10.0)),
                            .yaw = Angle(SimpleRandom::getDouble(-M_PI, M_PI)),
                            .targetAngularVelocity = SimpleRandom::getDouble(-5.0, 5.0),
                            .useAngularVelocity = SimpleRandom::getBool(),
                            .cameraYawOfRobot = Angle(SimpleRandom::getDouble(-M_PI, M_PI)),
                            .cameraYawOfRobotIsSet = SimpleRandom::getBool(),
                            .kickSpeed = SimpleRandom::getDouble(0.0, 5.0),
                            .waitForBall = SimpleRandom::getBool(),
                            .kickType = randomKickType(),
                            .kickAtYaw = SimpleRandom::getBool(),
                            .dribblerOn = SimpleRandom::getDouble(0.0, 5.0),
                            .wheelsOff = SimpleRandom::getBool()};
    RobotCommand copy = command;
    ASSERT_EQ(command, copy);
}
#include <RobotCommandsNetworker.hpp>
#include <RobotFeedbackNetworker.hpp>
#include <SettingsNetworker.hpp>
#include <WorldNetworker.hpp>
#include <chrono>
#include <iostream>
#include <memory>
#include <gtest/gtest.h>

using namespace rtt::net;

constexpr int TEST_VALUE = 69;

// Robot commands loop test
bool robotCommandsBlueLoopTestPassed = false;
bool robotCommandsYellowLoopTestPassed = false;
void onRobotCommandsBlue(const proto::AICommand& commands) { robotCommandsBlueLoopTestPassed = commands.commands().Get(0).id() == TEST_VALUE; }
void onRobotCommandsYellow(const proto::AICommand& commands) { robotCommandsYellowLoopTestPassed = commands.commands().Get(0).id() == TEST_VALUE; }

TEST(RTTChannels, testRobotCommandsLoop) {
    RobotCommandsBluePublisher pubBlue;
    RobotCommandsYellowPublisher pubYellow;

    auto callbackBlue = [](const proto::AICommand& commands) {
        onRobotCommandsBlue(commands);
    };
    auto callbackYellow = [](const proto::AICommand& commands) {
        onRobotCommandsYellow(commands);
    };
    RobotCommandsBlueSubscriber subBlue(onRobotCommandsBlue);
    RobotCommandsYellowSubscriber subYellow(onRobotCommandsYellow);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    proto::AICommand commands;
    proto::RobotCommand* command = commands.add_commands();
    command->set_id(TEST_VALUE);

    pubBlue.publish(commands);
    pubYellow.publish(commands);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));


    EXPECT_TRUE(robotCommandsBlueLoopTestPassed);
    EXPECT_TRUE(robotCommandsYellowLoopTestPassed);
}

// Robot feedback loop test
bool robotFeedbackLoopTestPassed = false;
void onRobotFeedback(const proto::RobotData& feedback) { robotFeedbackLoopTestPassed = feedback.receivedfeedback().Get(0).id() == TEST_VALUE; }
TEST(RTTChannels, testRobotFeedbackLoop) {
    RobotFeedbackPublisher pub;
    RobotFeedbackSubscriber sub(onRobotFeedback);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    proto::RobotData feedback;
    feedback.add_receivedfeedback()->set_id(TEST_VALUE);

    pub.publish(feedback);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    EXPECT_TRUE(robotFeedbackLoopTestPassed);
}

// Settings loop test
bool settingsLoopTestPassed = false;
void onSettings(const proto::Setting& settings) { settingsLoopTestPassed = settings.visionport() == TEST_VALUE; }
TEST(RTTChannels, testSettingsLoop) {
    SettingsPublisher pub;
    SettingsSubscriber sub(onSettings);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    proto::Setting settings;
    settings.set_visionport(TEST_VALUE);

    pub.publish(settings);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    EXPECT_TRUE(settingsLoopTestPassed);
}

// World loop test
bool worldLoopTestPassed = false;
void onWorld(const proto::State& world) { worldLoopTestPassed = world.ball_camera_world().ball().pos().x() == TEST_VALUE; }
TEST(RTTChannels, testWorldLoop) {
    WorldPublisher pub;
    WorldSubscriber sub(onWorld);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    proto::State world;
    world.mutable_ball_camera_world()->mutable_ball()->mutable_pos()->set_x(TEST_VALUE);

    pub.publish(world);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    EXPECT_TRUE(worldLoopTestPassed);
}
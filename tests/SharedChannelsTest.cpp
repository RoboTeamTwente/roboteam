#include <RobotCommandsNetworker.hpp>
#include <RobotFeedbackNetworker.hpp>
#include <SettingsNetworker.hpp>
#include <WorldNetworker.hpp>
#include <iostream>
#include <gtest/gtest.h>

using namespace rtt::net;

void sharedOnRobotCommands(const rtt::RobotCommands& command) {}
void sharedOnRobotsFeedback(const proto::RobotData& robotFeedback) {}
void sharedOnSettings(const proto::Setting& settings) {}
void sharedOnWorld(const proto::State& world) {}

TEST(RTTCMDPub, nullptrDontCrash) {
    EXPECT_NO_THROW(
        RobotCommandsBluePublisher blueCommandsPub;
        RobotCommandsBlueSubscriber blueCommandsSub1(sharedOnRobotCommands);
        RobotCommandsBlueSubscriber blueCommandsSub2(sharedOnRobotCommands);

        RobotCommandsYellowPublisher yellowCommandsPub;
        RobotCommandsYellowSubscriber yellowCommandsSub1(sharedOnRobotCommands);
        RobotCommandsYellowSubscriber yellowCommandsSub2(sharedOnRobotCommands);

        RobotFeedbackPublisher feedbackPub;
        RobotFeedbackSubscriber feedbackSub1(sharedOnRobotsFeedback);
        RobotFeedbackSubscriber feedbackSub2(sharedOnRobotsFeedback);

        SettingsPublisher settingsPub;
        SettingsSubscriber settingsSub1(sharedOnSettings);
        SettingsSubscriber settingsSub2(sharedOnSettings);

        WorldPublisher worldPub;
        WorldSubscriber worldSub1(sharedOnWorld);
        WorldSubscriber worldSub2(sharedOnWorld);
    );
}
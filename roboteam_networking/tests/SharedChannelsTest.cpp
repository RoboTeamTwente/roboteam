#include <gtest/gtest.h>

#include <RobotCommandsNetworker.hpp>
#include <RobotFeedbackNetworker.hpp>
#include <SettingsNetworker.hpp>
#include <WorldNetworker.hpp>
#include <iostream>

/* Each computer can only have a single publisher per channel.
 * To make sure all publishers do not accidentally share a channel,
 * this test will create all of them at the same time.
 * For completeness sake, this also takes into account multiple
 * subscribers.
 * */

using namespace rtt::net;

void sharedOnRobotCommands(const rtt::RobotCommands& command) {
}
void sharedOnRobotsFeedback(const rtt::RobotsFeedback& robotFeedback) {
}
void sharedOnSettings(const proto::GameSettings& settings) {
}
void sharedOnWorld(const proto::State& world) {
}

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
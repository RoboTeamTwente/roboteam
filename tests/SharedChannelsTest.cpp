#include <RobotCommandsNetworker.hpp>
#include <RobotFeedbackNetworker.hpp>
#include <SettingsNetworker.hpp>
#include <WorldNetworker.hpp>
#include <iostream>
#include <gtest/gtest.h>

using namespace rtt::net;

TEST(RTTCMDPub, nullptrDontCrash) {
    EXPECT_NO_THROW(
        RobotCommandsBluePublisher blueCommandsPub;
        RobotCommandsBlueSubscriber blueCommandsSub1(nullptr);
        RobotCommandsBlueSubscriber blueCommandsSub2(nullptr);

        RobotCommandsYellowPublisher yellowCommandsPub;
        RobotCommandsYellowSubscriber yellowCommandsSub1(nullptr);
        RobotCommandsYellowSubscriber yellowCommandsSub2(nullptr);

        RobotFeedbackPublisher feedbackPub;
        RobotFeedbackSubscriber feedbackSub1(nullptr);
        RobotFeedbackSubscriber feedbackSub2(nullptr);

        SettingsPublisher settingsPub;
        SettingsSubscriber settingsSub1(nullptr);
        SettingsSubscriber settingsSub2(nullptr);

        WorldPublisher worldPub;
        WorldSubscriber worldSub1(nullptr);
        WorldSubscriber worldSub2(nullptr);
    );
}
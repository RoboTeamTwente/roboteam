#include <RobotCommandsNetworker.hpp>
#include <RobotFeedbackNetworker.hpp>
#include <SettingsNetworker.hpp>
#include <WorldNetworker.hpp>

#include <iostream>

using namespace rtt::net;

int main() {
    try {
        RobotCommandsPublisher commandsPub;
        RobotCommandsSubscriber commandsSub1(nullptr);
        RobotCommandsSubscriber commandsSub2(nullptr);

        RobotFeedbackPublisher feedbackPub;
        RobotFeedbackSubscriber feedbackSub1(nullptr);
        RobotFeedbackSubscriber feedbackSub2(nullptr);

        SettingsPublisher settingsPub;
        SettingsSubscriber settingsSub1(nullptr);
        SettingsSubscriber settingsSub2(nullptr);

        WorldPublisher worldPub;
        WorldSubscriber worldSub1(nullptr);
        WorldSubscriber worldSub2(nullptr);

        std::cout << "Test passed!" << std::endl;
    } catch (std::exception e) {
        std::cout << "Test failed!" << std::endl;
    }
}
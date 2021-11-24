#include <RobotCommandsNetworker.hpp>
#include <RobotFeedbackNetworker.hpp>
#include <SettingsNetworker.hpp>

#include <iostream>
#include <memory>
#include <chrono>

using namespace rtt::net;

constexpr int TEST_VALUE = 69;

// Robot commands loop test
bool robotCommandsLoopTestPassed = false;
void onRobotCommands(const proto::RobotCommands& commands) {
    robotCommandsLoopTestPassed = commands.commands().Get(0).id() == TEST_VALUE;
}
bool testRobotCommandsLoop() {
    RobotCommandsPublisher pub;
    RobotCommandsSubscriber sub(onRobotCommands);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    proto::RobotCommands commands;
    proto::RobotCommand* command = commands.add_commands();
    command->set_id(TEST_VALUE);
    pub.publish(commands);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    return robotCommandsLoopTestPassed;
}

// Robot feedback loop test
bool robotFeedbackLoopTestPassed = false;
void onRobotFeedback(const proto::RobotFeedback& feedback) {
    robotFeedbackLoopTestPassed = feedback.basestation_robot_feedback().id() == TEST_VALUE;
}
bool testRobotFeedbackLoop() {
    RobotFeedbackPublisher pub;
    RobotFeedbackSubscriber sub(onRobotFeedback);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    proto::RobotFeedback feedback;
    feedback.mutable_basestation_robot_feedback()->set_id(TEST_VALUE);

    pub.publish(feedback);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    return robotFeedbackLoopTestPassed;
}

// Settings loop test
bool settingsLoopTestPassed = false;
void onSettings(const proto::Settings& settings) {
    settingsLoopTestPassed = settings.robothub_settings().bluecontrolport() == TEST_VALUE;
}
bool testSettingsLoop() {
    SettingsPublisher pub;
    SettingsSubscriber sub(onSettings);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    proto::Settings settings;
    settings.mutable_robothub_settings()->set_bluecontrolport(TEST_VALUE);

    pub.publish(settings);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    return settingsLoopTestPassed;
}

int main() {
    std::cout << "Starting..." << std::endl;
    std::cout << "RobotCommands loop test passed: " << (testRobotCommandsLoop() ? "true" : "false") << std::endl;
    std::cout << "RobotFeedback loop test passed: " << (testRobotFeedbackLoop() ? "true" : "false") << std::endl;
    std::cout << "Settings loop test passed: " << (testSettingsLoop() ? "true" : "false") << std::endl;

    std::cout << "Done!" << std::endl;

    return 0;
}
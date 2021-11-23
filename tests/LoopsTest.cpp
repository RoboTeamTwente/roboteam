#include <RobotCommandsNetworker.hpp>

#include <iostream>
#include <memory>
#include <chrono>

using namespace rtt::net;

constexpr int TEST_VALUE = 69;

// Robot commands testing
bool robotCommandsPassed = false;
void onRobotCommands(const proto::RobotCommands& commands) {
    robotCommandsPassed = commands.commands().Get(0).id() == TEST_VALUE;
}
bool testRobotCommands() {
    RobotCommandsPublisher pub;
    RobotCommandsSubscriber sub(onRobotCommands);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    proto::RobotCommands commands;
    proto::RobotCommand* command = commands.add_commands();
    command->set_id(TEST_VALUE);
    pub.publish(commands);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    return robotCommandsPassed;
}

int main() {
    std::cout << "Starting..." << std::endl;
    std::cout << "RobotCommands loop test passed: " << (testRobotCommands() ? "true" : "false") << std::endl;

    std::cout << "Done!" << std::endl;

    return 0;
}
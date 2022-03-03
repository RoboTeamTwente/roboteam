#include <RobotCommandsNetworker.hpp>
#include <RobotFeedbackNetworker.hpp>
#include <SettingsNetworker.hpp>
#include <WorldNetworker.hpp>
#include <chrono>
#include <iostream>
#include <memory>

using namespace rtt::net;

constexpr int ID = 0;
constexpr int MAX_AMOUNT_OF_ROBOTS = 16;

rtt::RobotCommands getEmptyRobotCommandToAllRobots() {
    rtt::RobotCommands commands;
    for (unsigned int i = 0; i < 16; ++i) {
        commands.push_back({.id = i});
    }
    return commands;
}

proto::Setting getRobotHubBasestationModePacket() {
    proto::Setting settings;

    settings.set_serialmode(true);

    return settings;
}

void onFeedback(const proto::RobotData& robotFeedback) {
    int amountOfFeedback = robotFeedback.receivedfeedback().size();
    bool isYellow = robotFeedback.isyellow();

    std::cout << "Received " << amountOfFeedback << " feedbacks of team " << (isYellow ? "yellow" : "blue") << std::endl;

    for (const auto& feedback : robotFeedback.receivedfeedback()) {
        int id = feedback.id();
        bool xSensCalibrated = feedback.xsenscalibrated();
        bool ballsensorworking = feedback.ballsensorisworking();
        bool hasBall = feedback.hasball();

        std::cout << " - Robot " << id << " has " << (hasBall ? "" : "not ") << "the ball" << std::endl;
    }
}

void onWorld(const proto::State& world) {
    for (const auto& robot : world.last_seen_world().yellowfeedback()) {
        // std::cout << "Robot " << robot.id() << " of team yellow has " << (robot.hasball() ? "" : "not ") << "the ball" << std::endl;
    }

    for (const auto& robot : world.last_seen_world().bluefeedback()) {
        // std::cout << "Robot " << robot.id() << " of team blue has " << (robot.hasball() ? "" : "not ") << "the ball" << std::endl;
    }
}

int main() {
    auto settingsPub = std::make_unique<SettingsPublisher>();
    auto commandsBluePub = std::make_unique<RobotCommandsBluePublisher>();
    auto commandsYellowPub = std::make_unique<RobotCommandsYellowPublisher>();

    auto feedbackSub = std::make_unique<RobotFeedbackSubscriber>(onFeedback);
    auto worldSub = std::make_unique<WorldSubscriber>(onWorld);

    auto teamCommand = getEmptyRobotCommandToAllRobots();

    // First, send settings message to robothub that it needs to forward messages to the basestation
    std::cout << "Started test..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(2));
    std::cout << "Sending robothub mode settings message" << std::endl;
    settingsPub->publish(getRobotHubBasestationModePacket());
    std::this_thread::sleep_for(std::chrono::seconds(2));

    std::cout << "Starting to send commands: " << std::endl << std::endl;
    auto command = getEmptyRobotCommandToAllRobots();
    while (true) {
        commandsYellowPub->publish(command);
        commandsBluePub->publish(command);
        std::cout << "Sent robot command..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}
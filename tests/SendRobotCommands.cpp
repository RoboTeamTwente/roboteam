#include <RobotCommandsNetworker.hpp>
#include <RobotFeedbackNetworker.hpp>
#include <SettingsNetworker.hpp>
#include <WorldNetworker.hpp>

#include <iostream>
#include <memory>
#include <chrono>

using namespace rtt::net;

void sendRobotHubBasestationMode(std::unique_ptr<SettingsPublisher>& pub) {
    proto::Setting settings;

    settings.set_serialmode(true);

    pub->publish(settings);
}

void sendEmptyRobotCommand(std::unique_ptr<RobotCommandsBluePublisher>& pub) {
    proto::AICommand commands;

    auto* command = commands.add_commands();
    command->set_id(0);
    command->mutable_vel()->set_x(0);
    command->mutable_vel()->set_y(0);
    command->set_w(0);
    command->set_use_angle(false);
    command->set_dribbler(false);
    command->set_kicker(false);
    command->set_chipper(false);
    command->set_chip_kick_forced(false);
    command->set_chip_kick_vel(false);

    pub->publish(commands);
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
        std::cout << "Robot " << robot.id() << " of team yellow has " << (robot.hasball() ? "" : "not ") << "the ball" << std::endl;
    }

    for (const auto& robot : world.last_seen_world().bluefeedback()) {
        std::cout << "Robot " << robot.id() << " of team blue has " << (robot.hasball() ? "" : "not ") << "the ball" << std::endl;
    }
}

int main() {

    std::unique_ptr<SettingsPublisher> settingsPub = std::make_unique<SettingsPublisher>();
    std::unique_ptr<RobotCommandsBluePublisher> commandsPub = std::make_unique<RobotCommandsBluePublisher>();
    
    auto callback = [](const proto::RobotData& data){
        onFeedback(data);
    };

    std::unique_ptr<RobotFeedbackSubscriber> feedbackSub = std::make_unique<RobotFeedbackSubscriber>(callback);
    std::unique_ptr<WorldSubscriber> worldSub = std::make_unique<WorldSubscriber>(onWorld);

    // First, send settings message to robothub that it needs to forward messages to the basestation
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    std::cout << "Sending robothub mode settings message" << std::endl;
    sendRobotHubBasestationMode(settingsPub);

    std::this_thread::sleep_for(std::chrono::seconds(2));


    while (true) {
        sendEmptyRobotCommand(commandsPub);
        std::cout << "Sent robot command..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}
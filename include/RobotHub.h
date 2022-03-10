#pragma once

#include <libusb-1.0/libusb.h>
#include <utilities.h>

#include <RobotCommandsNetworker.hpp>
#include <RobotFeedbackNetworker.hpp>
#include <SettingsNetworker.hpp>
#include <SimulationConfigurationNetworker.hpp>
#include <WorldNetworker.hpp>
#include <basestation/BasestationManager.hpp>
#include <exception>
#include <functional>
#include <memory>
#include <roboteam_utils/RobotCommands.hpp>
#include <roboteam_utils/RobotFeedback.hpp>
#include <simulation/SimulatorManager.hpp>

namespace rtt::robothub {

constexpr int MAX_AMOUNT_OF_ROBOTS = 16;

enum class RobotHubMode { NEITHER, SIMULATOR, BASESTATION, BOTH };

class RobotHub {
   public:
    RobotHub();

    void printStatistics();

   private:
    std::unique_ptr<simulation::SimulatorManager> simulatorManager;
    std::unique_ptr<basestation::BasestationManager> basestationManager;

    proto::Setting settings;
    utils::RobotHubMode mode;

    std::unique_ptr<rtt::net::RobotCommandsBlueSubscriber> robotCommandsBlueSubscriber;
    std::unique_ptr<rtt::net::RobotCommandsYellowSubscriber> robotCommandsYellowSubscriber;
    std::unique_ptr<rtt::net::SettingsSubscriber> settingsSubscriber;
    std::unique_ptr<rtt::net::RobotFeedbackPublisher> robotFeedbackPublisher;
    std::unique_ptr<rtt::net::SimulationConfigurationSubscriber> simulationConfigurationSubscriber;

    int commands_sent[MAX_AMOUNT_OF_ROBOTS] = {};
    int feedback_received[MAX_AMOUNT_OF_ROBOTS] = {};

    bool subscribe();

    void sendCommandsToSimulator(const rtt::RobotCommands &commands, utils::TeamColor color);
    void sendCommandsToBasestation(const rtt::RobotCommands &commands, utils::TeamColor color);

    void onBlueRobotCommands(const rtt::RobotCommands &commands);
    void onYellowRobotCommands(const rtt::RobotCommands &commands);
    void processRobotCommands(const rtt::RobotCommands &commands, utils::TeamColor color, utils::RobotHubMode mode);

    void onSettings(const proto::Setting &setting);

    void onSimulationConfiguration(const proto::SimulationConfiguration &configuration);

    void handleRobotFeedbackFromSimulator(const simulation::RobotControlFeedback &feedback);
    void handleRobotFeedbackFromBasestation(const REM_RobotFeedback &feedback, utils::TeamColor team);
    bool sendRobotFeedback(const rtt::RobotsFeedback &feedback);
};

class FailedToInitializeNetworkersException : public std::exception {
    virtual const char *what() const throw();
};

std::shared_ptr<proto::WorldRobot> getWorldBot(unsigned int id, utils::TeamColor color, const proto::World &world);

}  // namespace rtt::robothub
#pragma once

#include <libusb-1.0/libusb.h>

#include <RobotCommandsNetworker.hpp>
#include <RobotFeedbackNetworker.hpp>
#include <RobotHubStatistics.hpp>
#include <SettingsNetworker.hpp>
#include <SimulationConfigurationNetworker.hpp>
#include <WorldNetworker.hpp>
#include <basestation/BasestationManager.hpp>
#include <exception>
#include <functional>
#include <memory>
#include <roboteam_utils/RobotCommands.hpp>
#include <roboteam_utils/RobotFeedback.hpp>
#include <roboteam_utils/Teams.hpp>
#include <simulation/SimulatorManager.hpp>
#include <roboteam_utils/FileLogger.hpp>

namespace rtt::robothub {

constexpr int MAX_AMOUNT_OF_ROBOTS = 16;

enum class RobotHubMode { NEITHER, SIMULATOR, BASESTATION };

class RobotHub {
   public:
    RobotHub(bool shouldLog);

    const RobotHubStatistics &getStatistics();
    void resetStatistics();

   private:
    std::thread loggerThread;

    std::unique_ptr<FileLogger> robotStateLogger;
    std::unique_ptr<FileLogger> robotCommandLogger;
    std::unique_ptr<FileLogger> robotFeedbackLogger;

    std::unique_ptr<simulation::SimulatorManager> simulatorManager;
    std::unique_ptr<basestation::BasestationManager> basestationManager;

    proto::Setting settings;
    utils::RobotHubMode mode;

    RobotHubStatistics statistics;

    std::unique_ptr<rtt::net::RobotCommandsBlueSubscriber> robotCommandsBlueSubscriber;
    std::unique_ptr<rtt::net::RobotCommandsYellowSubscriber> robotCommandsYellowSubscriber;
    std::unique_ptr<rtt::net::SettingsSubscriber> settingsSubscriber;
    std::unique_ptr<rtt::net::RobotFeedbackPublisher> robotFeedbackPublisher;
    std::unique_ptr<rtt::net::SimulationConfigurationSubscriber> simulationConfigurationSubscriber;

    bool initializeNetworkers();

    void sendCommandsToSimulator(const rtt::RobotCommands &commands, rtt::Team color);
    void sendCommandsToBasestation(const rtt::RobotCommands &commands, rtt::Team color);

    std::mutex onRobotCommandsMutex;  // Guards the onRobotCommands function, as this can be called from two callback threads
    void onRobotCommands(const rtt::RobotCommands &commands, rtt::Team color);

    void onSettings(const proto::Setting &setting);

    void onSimulationConfiguration(const proto::SimulationConfiguration &configuration);

    void handleRobotFeedbackFromSimulator(const simulation::RobotControlFeedback &feedback);
    void handleRobotFeedbackFromBasestation(const REM_RobotFeedback &feedback, rtt::Team team);
    bool sendRobotFeedback(const rtt::RobotsFeedback &feedback);

    void handleSimulationConfigurationFeedback(const simulation::ConfigurationFeedback&);

    void handleRobotStateInfo(const REM_RobotStateInfo& robotStateInfo, rtt::Team team);

    void handleBasestationLog(const std::string& basestationLogMessage, rtt::Team team);

    void handleSimulationErrors(const std::vector<simulation::SimulationError>&);

    void logRobotStateInfo(const REM_RobotStateInfo& robotStateInfo, rtt::Team team);
    void logRobotCommands(const rtt::RobotCommands& commands, rtt::Team team);
    void logRobotFeedback(const rtt::RobotsFeedback& feedback);
};

class FailedToInitializeNetworkersException : public std::exception {
    virtual const char *what() const throw();
};

}  // namespace rtt::robothub
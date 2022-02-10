#pragma once

#include <libusb-1.0/libusb.h>
#include <utilities.h>

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
#include <simulation/SimulatorManager.hpp>

namespace rtt::robothub {

constexpr int MAX_AMOUNT_OF_ROBOTS = 16;

enum class RobotHubMode { NEITHER, SIMULATOR, BASESTATION };

class RobotHub {
   public:
    RobotHub();

    const RobotHubStatistics &getStatistics();
    void resetStatistics();

   private:
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

    void sendCommandsToSimulator(const proto::AICommand &commands, utils::TeamColor color);
    void sendCommandsToBasestation(const proto::AICommand &commands, utils::TeamColor color);

    std::mutex onRobotCommandsMutex;  // Guards the onRobotCommands function, as this can be called from two callback threads
    void onRobotCommands(const proto::AICommand &commands, utils::TeamColor color);

    void onSettings(const proto::Setting &setting);

    void onSimulationConfiguration(const proto::SimulationConfiguration &configuration);

    void handleRobotFeedbackFromSimulator(const simulation::RobotControlFeedback &feedback);
    void handleRobotFeedbackFromBasestation(const RobotFeedback &feedback, utils::TeamColor team);
    bool sendRobotFeedback(const proto::RobotData &feedback) const;
};

class FailedToInitializeNetworkersException : public std::exception {
    virtual const char *what() const throw();
};

std::shared_ptr<proto::WorldRobot> getWorldBot(unsigned int id, utils::TeamColor color, const proto::World &world);

}  // namespace rtt::robothub
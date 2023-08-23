#pragma once

#include <libusb-1.0/libusb.h>

// #include <RobotHubLogger.hpp>
#include <RobotHubMode.h>

#include <RobotCommandsNetworker.hpp>
#include <RobotFeedbackNetworker.hpp>
#include <RobotHubStatistics.hpp>
#include <SettingsNetworker.hpp>
#include <WorldNetworker.hpp>
#include <basestation/BasestationManager.hpp>
#include <exception>
#include <functional>
#include <memory>
#include <optional>
#include <roboteam_utils/FileLogger.hpp>
#include <roboteam_utils/RobotCommands.hpp>
#include <roboteam_utils/RobotFeedback.hpp>
#include <roboteam_utils/Teams.hpp>
#include <simulation/SimulatorManager.hpp>

namespace rtt::robothub {

    class RobotHub {
    public:
        explicit RobotHub(bool shouldLog, bool logInMarpleFormat = false);

        const RobotHubStatistics &getStatistics();
        void resetStatistics();

    private:
        // std::optional<RobotHubLogger> logger;

        std::unique_ptr<simulation::SimulatorManager> simulatorManager;
        std::unique_ptr<basestation::BasestationManager> basestationManager;

        proto::GameSettings settings;
        rtt::net::RobotHubMode mode = rtt::net::RobotHubMode::UNKNOWN;

        RobotHubStatistics statistics;

        std::unique_ptr<rtt::net::RobotCommandsBlueSubscriber> robotCommandsBlueSubscriber;
        std::unique_ptr<rtt::net::RobotCommandsYellowSubscriber> robotCommandsYellowSubscriber;
        std::unique_ptr<rtt::net::SettingsSubscriber> settingsSubscriber;
        std::unique_ptr<rtt::net::RobotFeedbackPublisher> robotFeedbackPublisher;

        bool initializeNetworkers();

        void sendCommandsToSimulator(const rtt::RobotCommands &commands, rtt::Team color);
        void sendCommandsToBasestation(const rtt::RobotCommands &commands, rtt::Team color);

        std::mutex onRobotCommandsMutex;  // Guards the onRobotCommands function, as this can be called from two callback threads
        void onRobotCommands(const rtt::RobotCommands &commands, rtt::Team color);

        void onSettings(const proto::GameSettings &setting);

        void handleRobotFeedbackFromSimulator(const simulation::RobotControlFeedback &feedback);
        void handleRobotFeedbackFromBasestation(const REM_RobotFeedback &feedback, rtt::Team team);
        bool sendRobotFeedback(const rtt::RobotsFeedback &feedback);

        void handleRobotStateInfo(const REM_RobotStateInfo &robotStateInfo, rtt::Team team);

        void handleBasestationLog(const std::string &basestationLogMessage, rtt::Team team);

        void handleSimulationErrors(const std::vector<simulation::SimulationError> &);
    };

    class FailedToInitializeNetworkersException : public std::exception {
        [[nodiscard]] const char *what() const noexcept override;
    };

}  // namespace rtt::robothub
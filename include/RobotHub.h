#pragma once

#include <constants.h>
#include <libusb-1.0/libusb.h>

#include <RobotCommandsNetworker.hpp>
#include <SettingsNetworker.hpp>
#include <WorldNetworker.hpp>
#include <RobotFeedbackNetworker.hpp>

#include <proto/RobotCommands.pb.h>
#include <proto/RobotFeedback.pb.h>
#include <proto/Settings.pb.h>
#include <utilities.h>
#include <memory>
#include <functional>
#include <basestation/BasestationManager.hpp>
#include <simulation/SimulatorManager.hpp>

namespace rtt::robothub {

enum class RobotHubMode {
    NEITHER, SIMULATOR, BASESTATION, BOTH
};

class RobotHub {
   public:
    RobotHub();

    void printStatistics();

   private:
    std::unique_ptr<simulation::SimulatorManager> simulatorManager;
    std::unique_ptr<basestation::BasestationManager> basestationManager;

    proto::Settings settings;
    RobotHubMode mode;
    proto::World world;

    std::unique_ptr<rtt::net::RobotCommandsBlueSubscriber> robotCommandsBlueSubscriber;
    std::unique_ptr<rtt::net::RobotCommandsYellowSubscriber> robotCommandsYellowSubscriber;
    std::unique_ptr<rtt::net::SettingsSubscriber> settingsSubscriber;
    std::unique_ptr<rtt::net::WorldSubscriber> worldSubscriber;
    std::unique_ptr<rtt::net::RobotFeedbackPublisher> robotFeedbackPublisher;

    int commands_sent[MAX_AMOUNT_OF_ROBOTS] = {};
    int feedback_received[MAX_AMOUNT_OF_ROBOTS] = {};

    void subscribe();

    void sendCommandsToSimulator(const proto::RobotCommands &commands, bool toTeamYellow);
    void sendCommandsToBasestation(const proto::RobotCommands &commands, bool toTeamYellow);

    void onBlueRobotCommands(const proto::RobotCommands& commands);
    void onYellowRobotCommands(const proto::RobotCommands& commands);
    void processRobotCommands(const proto::RobotCommands &commands, bool forTeamYellow, RobotHubMode mode);

    void onSettingsFromChannel1(const proto::Settings &setting);

    void handleRobotFeedbackFromSimulator(const simulation::RobotControlFeedback &feedback);
    void handleRobotFeedbackFromBasestation(const RobotFeedback &feedback);
};

}  // namespace rtt::robothub
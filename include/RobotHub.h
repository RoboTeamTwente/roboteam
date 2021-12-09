#pragma once

#include <constants.h>
#include <libusb-1.0/libusb.h>
#include <networking/Publisher.h>
#include <networking/Subscriber.h>
#include <roboteam_proto/AICommand.pb.h>
#include <roboteam_proto/RobotData.pb.h>
#include <roboteam_proto/Setting.pb.h>
#include <roboteam_proto/State.pb.h>
#include <utilities.h>

#include <basestation/BasestationManager.hpp>
#include <simulation/SimulatorManager.hpp>

namespace rtt::robothub {

class RobotHub {
   public:
    RobotHub();

    void printStatistics();

   private:
    std::unique_ptr<simulation::SimulatorManager> simulatorManager;
    std::unique_ptr<basestation::BasestationManager> basestationManager;

    proto::Setting settingsFromChannel1;
    proto::Setting settingsFromChannel2;

    proto::ChannelType robotCommandChannel;
    proto::ChannelType settingsChannel;

    std::unique_ptr<proto::Subscriber<proto::AICommand>> robotCommandSubscriber;
    std::unique_ptr<proto::Subscriber<proto::AICommand>> robotCommandSubscriber2;
    std::unique_ptr<proto::Subscriber<proto::Setting>> settingsSubscriber;
    std::unique_ptr<proto::Subscriber<proto::Setting>> settingsSubscriber2;
    std::unique_ptr<proto::Publisher<proto::RobotData>> feedbackPublisher;

    int commands_sent[MAX_AMOUNT_OF_ROBOTS] = {};
    int feedback_received[MAX_AMOUNT_OF_ROBOTS] = {};

    void subscribe();

    void sendCommandsToSimulator(const proto::AICommand &commands, bool toTeamYellow);
    void sendCommandsToBasestation(const proto::AICommand &commands, bool toTeamYellow);

    void onRobotCommandsFromChannel1(proto::AICommand &commands);
    void onRobotCommandsFromChannel2(proto::AICommand &commands);
    void processRobotCommands(proto::AICommand &commands, bool forTeamYellow, bool useBasestation);

    void onSettingsFromChannel1(proto::Setting &setting);
    void onSettingsFromChannel2(proto::Setting &setting);

    void handleRobotFeedbackFromSimulator(const simulation::RobotControlFeedback &feedback);
    void handleRobotFeedbackFromBasestation(const RobotFeedback &feedback);
};

}  // namespace rtt::robothub
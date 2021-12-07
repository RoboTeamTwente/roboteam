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

    proto::Setting settings;
    proto::Setting settings2;
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

    void sendCommandsToSimulator(const proto::AICommand &aiCmd, bool isForTeamYellow);
    void sendCommandsToBasestation(const proto::AICommand &aiCmd);

    void processAIcommand(proto::AICommand &AIcmd);
    void processAIcommand2(proto::AICommand &AIcmd);
    void processSettings(proto::Setting &setting);
    void processSettings2(proto::Setting &setting);

    void handleRobotFeedbackFromSimulator(const simulation::RobotControlFeedback &feedback);
    void handleRobotFeedbackFromBasestation(const RobotFeedback &feedback);
};

}  // namespace rtt::robothub
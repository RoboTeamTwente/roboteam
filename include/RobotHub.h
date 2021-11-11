#pragma once

#include <constants.h>
#include <libusb-1.0/libusb.h>
#include <networking/Publisher.h>
#include <networking/Subscriber.h>
#include <roboteam_proto/AICommand.pb.h>
#include <roboteam_proto/RobotFeedback.pb.h>
#include <roboteam_proto/Setting.pb.h>
#include <roboteam_proto/State.pb.h>
#include <utilities.h>

#include <basestation/BasestationManager.hpp>
#include <simulation/SimulatorManager.hpp>

namespace rtt::robothub {

void handleRobotFeedbackFromSimulator(const simulation::RobotControlFeedback &feedback);
void handleRobotFeedbackFromBasestation(const RobotFeedback &feedback);

class RobotHub {
   public:
    RobotHub();

    void printStatistics();

   private:
    std::unique_ptr<simulation::SimulatorManager> simulatorManager;
    std::unique_ptr<basestation::BasestationManager> basestationManager;

    proto::Setting settings;
    proto::ChannelType robotCommandChannel;
    proto::ChannelType settingsChannel;

    std::unique_ptr<proto::Subscriber<proto::AICommand>> robotCommandSubscriber;
    // std::unique_ptr<proto::Subscriber<proto::State>> worldStateSubscriber;
    std::unique_ptr<proto::Subscriber<proto::Setting>> settingsSubscriber;
    std::unique_ptr<proto::Publisher<proto::RobotFeedback>> feedbackPublisher;

    // std::mutex worldLock;
    proto::World world;

    int commands_sent[MAX_AMOUNT_OF_ROBOTS] = {};
    int feedback_received[MAX_AMOUNT_OF_ROBOTS] = {};

    void subscribe();

    void sendCommandsToSimulator(const proto::AICommand &aiCmd);
    void sendCommandsToBasestation(const proto::AICommand &aiCmd);

    void processAIcommand(proto::AICommand &AIcmd);
    void processSettings(proto::Setting &setting);
    // void processWorldState(proto::State &state);
};

}  // namespace rtt::robothub
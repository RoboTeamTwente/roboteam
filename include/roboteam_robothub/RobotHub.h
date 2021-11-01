//
// Created by mrlukasbos on 8-3-19.
//

#ifndef ROBOTEAM_ROBOTHUB_APPLICATION_H
#define ROBOTEAM_ROBOTHUB_APPLICATION_H

#include <networking/Publisher.h>
#include <networking/Subscriber.h>
#include <roboteam_proto/AICommand.pb.h>
#include <roboteam_proto/RobotData.pb.h>
#include <roboteam_proto/Setting.pb.h>

#include <string>

#include "SSLSimulator.h"
#include "constants.h"
#include "utilities.h"

namespace rtt::robothub {

class GRSimCommander;
class SerialDeviceManager;
class RobotHub {
   public:
    RobotHub();
    void start();
    void subscribeToTopics();

   private:
    utils::Mode mode = utils::Mode::GRSIM;
    bool isLeft = true;
    bool isYellow = true;

    proto::ChannelType robotCommandChannel;
    proto::ChannelType settingsChannel;

   public:
    void set_settings_channel(const proto::ChannelType &settings_channel);

   public:
    void set_robot_command_channel(const proto::ChannelType &robot_command_channel);
    void set_feedback_channel(const proto::ChannelType &feedback_channel);

   private:
    proto::ChannelType feedbackChannel;

   private:
    proto::Subscriber<proto::AICommand> *robotCommandSubscriber;
    proto::Publisher<proto::RobotData> *feedbackPublisher;
    proto::Subscriber<proto::Setting> *settingsSubscriber;

    // Callback functions

    void processAIBatch(proto::AICommand &cmd);
    void sendSimulatorBatch(proto::AICommand &cmd, const proto::World &world);
    bool processCommand(const proto::RobotCommand &robotCommand,const proto::World& world);
    void processSettings(proto::Setting &setting);

    // Serial and grsim managers
    std::shared_ptr<SerialDeviceManager> device;
    std::shared_ptr<GRSimCommander> grsimCommander;
    std::shared_ptr<SSLSimulator> simulator_connection;

    bool sendSerialCommand(LowLevelRobotCommand llrc);
    bool sendGrSimCommand(const proto::RobotCommand &robotCommand);
    void publishRobotFeedback(LowLevelRobotFeedback llrf);
    int robotTicks[MAX_AMOUNT_OF_ROBOTS] = {};
    void printStatistics();
    std::mutex worldLock;
};

}  // namespace rtt

#endif  // ROBOTEAM_ROBOTHUB_APPLICATION_H

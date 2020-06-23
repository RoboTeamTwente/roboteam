//
// Created by mrlukasbos on 8-3-19.
//

#ifndef ROBOTEAM_ROBOTHUB_APPLICATION_H
#define ROBOTEAM_ROBOTHUB_APPLICATION_H

#include <roboteam_proto/Publisher.h>
#include <roboteam_proto/Subscriber.h>
#include <string>
#include "constants.h"
#include "roboteam_proto/Setting.pb.h"
#include "utilities.h"

namespace rtt {
namespace robothub {

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
    proto::Subscriber<proto::RobotCommand> *robotCommandSubscriber;
    proto::Subscriber<proto::World> *worldStateSubscriber;
    proto::Subscriber<proto::Setting> *settingsSubscriber;
    proto::Publisher<proto::RobotFeedback> *feedbackPublisher;

    // Callback functions
    proto::World LastWorld;
    void processWorldState(proto::World &world);
    void processRobotCommand(proto::RobotCommand &cmd);
    void processSettings(proto::Setting &setting);

    // Serial and grsim managers
    std::shared_ptr<SerialDeviceManager> device;
    std::shared_ptr<GRSimCommander> grsimCommander;

    void sendSerialCommand(LowLevelRobotCommand llrc);
    void sendGrSimCommand(const proto::RobotCommand &robotCommand);
    void publishRobotFeedback(LowLevelRobotFeedback llrf);
    int robotTicks[MAX_AMOUNT_OF_ROBOTS] = {};
    void printStatistics();
    std::mutex worldLock;
};

}  // namespace robothub
}  // namespace rtt

#endif  // ROBOTEAM_ROBOTHUB_APPLICATION_H

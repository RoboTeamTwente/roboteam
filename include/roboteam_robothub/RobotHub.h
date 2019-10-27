//
// Created by mrlukasbos on 8-3-19.
//

#ifndef ROBOTEAM_ROBOTHUB_APPLICATION_H
#define ROBOTEAM_ROBOTHUB_APPLICATION_H

#include <string>
#include <roboteam_utils/constants.h>
#include "utilities.h"
#include "roboteam_proto/Setting.pb.h"
#include "constants.h"
#include <roboteam_proto/Subscriber.h>
#include <roboteam_proto/Publisher.h>

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

    roboteam_utils::ChannelType robotCommandChannel;
  roboteam_utils::ChannelType settingsChannel;
 public:
  void set_settings_channel(const roboteam_utils::ChannelType &settings_channel);

 public:
  void set_robot_command_channel(const roboteam_utils::ChannelType &robot_command_channel);
  void set_feedback_channel(const roboteam_utils::ChannelType &feedback_channel);
 private:
  roboteam_utils::ChannelType feedbackChannel;

 private:

  roboteam_proto::Subscriber<roboteam_proto::RobotCommand> * robotCommandSubscriber;
    roboteam_proto::Subscriber<roboteam_proto::World> * worldStateSubscriber;
    roboteam_proto::Subscriber<roboteam_proto::Setting> * settingsSubscriber;
    roboteam_proto::Publisher<roboteam_proto::RobotFeedback> * feedbackPublisher;

    // Callback functions
    roboteam_proto::World LastWorld;
    void processWorldState(roboteam_proto::World & world);
    void processRobotCommand(roboteam_proto::RobotCommand & cmd);
    void processSettings(roboteam_proto::Setting & setting);

    // Serial and grsim managers
    std::shared_ptr<SerialDeviceManager> device;
    std::shared_ptr<GRSimCommander> grsimCommander;

    void sendSerialCommand(LowLevelRobotCommand llrc);
    void sendGrSimCommand(const roboteam_proto::RobotCommand& robotCommand);
    void publishRobotFeedback(LowLevelRobotFeedback llrf);
    int robotTicks[MAX_AMOUNT_OF_ROBOTS] = {};
    void printStatistics();
    std::mutex worldLock;
};

} // robothub
} // rtt

#endif //ROBOTEAM_ROBOTHUB_APPLICATION_H

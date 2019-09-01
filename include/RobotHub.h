//
// Created by mrlukasbos on 8-3-19.
//

#ifndef ROBOTEAM_ROBOTHUB_APPLICATION_H
#define ROBOTEAM_ROBOTHUB_APPLICATION_H

#include <string>
#include <roboteam_utils/constants.h>
#include "utilities.h"
#include "Setting.pb.h"

#include "constants.h"
#include <Subscriber.h>
#include <Publisher.h>

namespace rtt {
namespace robothub {

class GRSimCommander;
class SerialDeviceManager;
class RobotHub {
public:
  RobotHub();
    void loop();
private:
    std::string ai_publisher;
    std::string robothub_publisher;
public:
    void setRobothubPublisher(const string &robothubPublisher);

public:
    void setAiPublisher(const string &aiPublisher);
    void subscribeToROSTopics();

private:
    utils::Mode mode = utils::Mode::GRSIM;


  roboteam_proto::Subscriber * robotCommandSubscriber;
  roboteam_proto::Subscriber * worldStateSubscriber;
    roboteam_proto::Subscriber * settingsSubscriber;
    roboteam_proto::Publisher * publisher;

    // get parameters from ROS
    utils::Mode getMode();
    void setMode();
    std::string getSerialDevice();
    bool getBatchingVariable();

    // ROS callback functions
    roboteam_proto::World LastWorld;
    void processWorldState(roboteam_proto::World & world);
    void processRobotCommand(roboteam_proto::RobotCommand & cmd);
    void processSettings(roboteam_proto::Setting & setting);
    roboteam_proto::Setting settings;

    // Serial and grsim managers
    std::shared_ptr<SerialDeviceManager> device;
    std::shared_ptr<GRSimCommander> grsimCommander;

    void sendSerialCommand(LowLevelRobotCommand llrc);
    void sendGrSimCommand(const roboteam_proto::RobotCommand& robotCommand);
    void publishRobotFeedback(LowLevelRobotFeedback llrf);

    int robotTicks[MAX_AMOUNT_OF_ROBOTS] = {};
    void printStatistics();
    std::string target;

    bool batching = false;

    std::mutex worldLock;
};

} // robothub
} // rtt

#endif //ROBOTEAM_ROBOTHUB_APPLICATION_H

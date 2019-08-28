//
// Created by mrlukasbos on 8-3-19.
//

#ifndef ROBOTEAM_ROBOTHUB_APPLICATION_H
#define ROBOTEAM_ROBOTHUB_APPLICATION_H

#include <string>
#include "utilities.h"
#include "constants.h"

namespace rtt {
namespace robothub {

class GRSimCommander;
class SerialDeviceManager;
class RobotHub {
public:
  RobotHub();
    void loop();
private:
    utils::Mode mode = utils::Mode::UNDEFINED;

    // ROS subscriptions
//    ros::NodeHandle n;
//    ros::Subscriber subWorldState;
//    ros::Subscriber subRobotCommands;
//    ros::Publisher feedbackPublisher;
    void subscribeToROSTopics();

    // get parameters from ROS
    utils::Mode getMode();
    void setMode();
    std::string getSerialDevice();
    bool getBatchingVariable();

    // ROS callback functions
    std::shared_ptr<roboteam_proto::World> LastWorld;
    void processWorldState(const roboteam_proto::World& world);
    void processRobotCommand(const roboteam_proto::RobotCommand& cmd);


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
};

} // robothub
} // rtt

#endif //ROBOTEAM_ROBOTHUB_APPLICATION_H

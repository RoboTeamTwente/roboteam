//
// Created by mrlukasbos on 8-3-19.
//

#ifndef ROBOTEAM_ROBOTHUB_APPLICATION_H
#define ROBOTEAM_ROBOTHUB_APPLICATION_H

#include <string>
#include <ros/subscriber.h>
#include <ros/node_handle.h>
#include "utilities.h"
#include "constants.h"

namespace rtt {
namespace robothub {

class GRSimCommander;
class SerialDeviceManager;
class Application {
public:
    Application();
    void loop();
private:
    utils::Mode mode = utils::Mode::UNDEFINED;

    // ROS subscriptions
    ros::NodeHandle n;
    ros::Subscriber subWorldState;
    ros::Subscriber subRobotCommands;
    void subscribeToROSTopics();

    // get parameters from ROS
    utils::Mode getMode();
    void setMode();
    std::string getSerialDevice();
    bool getBatchingVariable();

    // ROS callback functions
    std::shared_ptr<roboteam_msgs::World> LastWorld;
    void processWorldState(const roboteam_msgs::World& world);
    void processRobotCommand(const roboteam_msgs::RobotCommand& cmd);

    // Serial and grsim managers
    std::shared_ptr<SerialDeviceManager> device;
    std::shared_ptr<GRSimCommander> grsimCommander;

    void sendSerialCommand(LowLevelRobotCommand llrc);
    void sendGrSimCommand(const roboteam_msgs::RobotCommand& robotCommand);

    int robotTicks[MAX_AMOUNT_OF_ROBOTS] = {};
    void printStatistics();
    std::string target;

    bool batching = false;
};

} // robothub
} // rtt

#endif //ROBOTEAM_ROBOTHUB_APPLICATION_H

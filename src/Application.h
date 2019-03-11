//
// Created by mrlukasbos on 8-3-19.
//

#ifndef ROBOTEAM_ROBOTHUB_APPLICATION_H
#define ROBOTEAM_ROBOTHUB_APPLICATION_H

#include <string>
#include <ros/subscriber.h>
#include <ros/node_handle.h>
#include "utilities.h"

namespace rtt {
namespace robothub {

class SerialDeviceManager;
class Application {
public:
    Application();
    utils::Mode getMode();
    std::string getSerialDevice();
    std::shared_ptr<SerialDeviceManager> device;
    void subscribeToROSTopics();
    void loop();
private:
    utils::Mode mode = utils::Mode::UNDEFINED;
    ros::Subscriber subWorldState;
    ros::Subscriber subRobotCommands;
    ros::NodeHandle n;

    std::shared_ptr<roboteam_msgs::World> LastWorld;

    void processWorldState(const roboteam_msgs::World& world);
    void processRobotCommand(const roboteam_msgs::RobotCommand& cmd);

    void sendSerialCommand(const roboteam_msgs::RobotCommand& robotCommand);
    void sendGrSimCommand(const roboteam_msgs::RobotCommand& robotCommand);

    std::string StandardDeviceNames[3] = {
            "/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_00000000001A-if00",
            "/dev/serial/by-id/usb-STMicroelectronics_Basestation_078-if00",
            "/dev/serial/by-id/usb-STMicroelectronics_Basestation_080-if00",
    };

    rtt::GRSimCommander grsimCmd(true);
    SlowParam<bool> grSimBatch("grsim/batching", true);

    std::string serial_file_path_param = "none";
    std::string serial_file_path = "No basestation selected!";
};

}
}

#endif //ROBOTEAM_ROBOTHUB_APPLICATION_H

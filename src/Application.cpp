
#include <ros/param.h>
#include "Application.h"
#include "SerialDeviceManager.h"
#include "GRSim.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_msgs/RobotFeedback.h"
#include "packing.h"
#include "constants.h"

namespace rtt {
namespace robothub {

Application::Application() {
    subscribeToROSTopics();
    batching = getBatchingVariable();

    // if you want to force a mode, set it here already
    mode = utils::Mode::GRSIM;

    // set up the managers
    grsimCommander = std::make_shared<GRSimCommander>();
    if (getMode() == utils::Mode::SERIAL) {
        device = std::make_shared<SerialDeviceManager>(getSerialDevice());
    }
}

/// Get the mode of robothub, either "serial" or "grsim" or "undefined"
utils::Mode Application::getMode() {
    if (mode == utils::Mode::UNDEFINED) {
        std::string modeStr = "undefined";
        ros::param::get("robot_output_target", modeStr);
        mode = rtt::robothub::utils::stringToMode(modeStr);
        ROS_INFO_STREAM("Current mode : " << utils::modeToString(mode));
    }
    return mode;
}

/// Get output device from the ROS parameter server
// if there is no serial device from ROS then auto select one
// then always just open basestation 78, which is the default (no jumper needed)
std::string Application::getSerialDevice() {
    std::string deviceName;

    if(ros::param::has("output_device")) {
        ros::param::get("output_device", deviceName);
        if (deviceName != "none") {
            ROS_INFO_STREAM("[getSerialDevice] Looking for serial device defined by ROS parameter: " << deviceName);
            return deviceName;
        }
    }

    deviceName = "/dev/serial/by-id/usb-STMicroelectronics_Basestation_078-if00";
    ROS_INFO_STREAM("[getSerialDevice] No serial device name given in ROS. Assuming basestation " << deviceName);
    return deviceName;
}

/// Get batching mode for grsim from the ROS parameter server
bool Application::getBatchingVariable() {
    std::string batching = "true";
    if(ros::param::has("grsim/batching")) {
        ros::param::get("grsim/batching", batching);
        if (batching == "false") return false;
    }
    return true;
}

/// subscribe to ROS topics
void Application::subscribeToROSTopics(){
    subWorldState = n.subscribe("world_state", 1000, &Application::processWorldState, this, ros::TransportHints().tcpNoDelay());
    subRobotCommands = n.subscribe("robotcommands", 1000, &Application::processRobotCommand, this,  ros::TransportHints().tcpNoDelay());
}


void Application::loop(){
    ros::Rate rate(TICK_RATE);
    std::chrono::high_resolution_clock::time_point lastStatistics = std::chrono::high_resolution_clock::now();

    grsimCommander->setBatch(batching);


    int currIteration = 0;
    while (ros::ok()) {
        rate.sleep();
        ros::spinOnce();
        auto timeNow = std::chrono::high_resolution_clock::now();
        auto timeDiff = timeNow-lastStatistics;
        if (std::chrono::duration_cast<std::chrono::milliseconds>(timeDiff).count()>1000) {
            lastStatistics = timeNow;
            ROS_INFO_STREAM("==========| " << currIteration++ << " |==========");
            printStatistics();
        }
    }
}

/// print robot ticks in a nicely formatted way
void Application::printStatistics() {
    const int amountOfColumns = 4;
    for (int i = 0; i < MAX_AMOUNT_OF_ROBOTS; i+=amountOfColumns) {
        for (int j = 0; j < amountOfColumns; j++) {
            const int robotId = i+j;
            if (robotId < MAX_AMOUNT_OF_ROBOTS) {
                std::cout << robotId << ": " << robotTicks[robotId] << "\t";
                robotTicks[robotId] = 0;
            }
        }
        std::cout << std::endl;
    }
}

void Application::processWorldState(const roboteam_msgs::World& world){
    LastWorld = std::make_shared<roboteam_msgs::World>(world);
}

void Application::processRobotCommand(const roboteam_msgs::RobotCommand& cmd) {
    LowLevelRobotCommand llrc = createLowLevelRobotCommand(cmd, LastWorld);

    // check if the command is valid, otherwise don't send anything
    if(!validateRobotPacket(llrc)) {
        ROS_ERROR_STREAM("[processRobotCommand] LowLevelRobotCommand is not valid for our robots, no command is being sent!");
        printLowLevelRobotCommand(llrc);
        return;
    }

    robotTicks[cmd.id]++;
    if (getMode() == utils::Mode::SERIAL) {
        sendSerialCommand(llrc);
    } else {
        sendGrSimCommand(cmd);
    }
}

/// send a serial command from a given robotcommand
void Application::sendSerialCommand(LowLevelRobotCommand llrc) {

    // convert the LLRC to a bytestream which we can send
    std::shared_ptr<packed_protocol_message> bytestream = createRobotPacket(llrc);

    // Check if the message was created successfully
    if(!bytestream){
        ROS_ERROR("[sendSerialCommand] The message was not created succesfully!");
        return;
    }
    packed_protocol_message packet = *bytestream;
    device->writeToDevice(packet);
}

/// send a GRSim command from a given robotcommand
void Application::sendGrSimCommand(const roboteam_msgs::RobotCommand& robotCommand) {
        this->grsimCommander->queueGRSimCommand(robotCommand);
}

} // robothub
} // rtt

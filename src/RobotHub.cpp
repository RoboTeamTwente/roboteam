
#include "../include/RobotHub.h"
#include "../include/SerialDeviceManager.h"
#include "../include/GRSim.h"
#include "../include/packing.h"
#include "../include/constants.h"

namespace rtt {
namespace robothub {

RobotHub::RobotHub() {
    subscribeToROSTopics();
    setMode();
    batching = getBatchingVariable();

    // set up the managers
    grsimCommander = std::make_shared<GRSimCommander>();
    if (getMode() == utils::Mode::SERIAL) {
        device = std::make_shared<SerialDeviceManager>(getSerialDevice());

    }
    if (getMode() == utils::Mode::SERIAL && !device->ensureDeviceOpen())
        device->openDevice();

}

/// Get the mode of robothub, either "serial" or "grsim" or "undefined"
utils::Mode RobotHub::getMode() {
    if (mode == utils::Mode::UNDEFINED) {
        std::string modeStr = "undefined";
       // ros::param::get("robot_output_target", modeStr);
        mode = rtt::robothub::utils::stringToMode(modeStr);
        std::cout << "Current mode : " << utils::modeToString(mode) << std::endl;
    }
    return mode;
}

/// Get output device from the ROS parameter server
// if there is no serial device from ROS then auto select one
// then always just open basestation 78, which is the default (no jumper needed)
std::string RobotHub::getSerialDevice() {
    std::string deviceName;
//
//    if(ros::param::has("output_device")) {
//        ros::param::get("output_device", deviceName);
//        if (deviceName != "none") {
//            std::cout << "[getSerialDevice] Looking for serial device defined by ROS parameter: " << deviceName << std::endl;
//            return deviceName;
//        }
//    }

    deviceName = "/dev/serial/by-id/usb-RTT_BaseStation_00000000001A-if00";
    std::cout << "[getSerialDevice] No serial device name given in ROS. Assuming basestation " << deviceName << std::endl;
    return deviceName;
}

/// Get batching mode for grsim from the ROS parameter server
bool RobotHub::getBatchingVariable() {
    std::string batching = "true";
//    if(ros::param::has("grsim/batching")) {
//        ros::param::get("grsim/batching", batching);
//        if (batching == "false") return false;
//    }
    return true;
}

/// subscribe to ROS topics
void RobotHub::subscribeToROSTopics(){
//    feedbackPublisher = n.advertise<roboteam_proto::RobotFeedback>("robot_feedback", 1000);
//    subWorldState = n.subscribe("world_state", 1000, &Application::processWorldState, this, ros::TransportHints().tcpNoDelay());
//    subRobotCommands = n.subscribe("robotcommands", 1000, &Application::processRobotCommand, this,  ros::TransportHints().tcpNoDelay());
//    n.getParam("robot_output_target", target);
//    std::cout << "Set the target to: " << target << std::endl;
}


void RobotHub::loop(){
 //   ros::Rate rate(ROBOTHUB_TICK_RATE);
    std::chrono::high_resolution_clock::time_point lastStatistics = std::chrono::high_resolution_clock::now();

    grsimCommander->setBatch(batching);

    int currIteration = 0;
    while (true) { // ros::ok()
//        rate.sleep();
//        ros::spinOnce();
        auto timeNow = std::chrono::high_resolution_clock::now();
        auto timeDiff = timeNow-lastStatistics;
        if (std::chrono::duration_cast<std::chrono::milliseconds>(timeDiff).count()>1000) {
            lastStatistics = timeNow;
            std::cout << "==========| " << currIteration++ << "   " << utils::modeToString(getMode()) << " |==========" << std::endl;
            printStatistics();
        }

    }
}

/// print robot ticks in a nicely formatted way
void RobotHub::printStatistics() {
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

void RobotHub::processWorldState(const roboteam_proto::World& world){
    LastWorld = std::make_shared<roboteam_proto::World>(world);
}

void RobotHub::processRobotCommand(const roboteam_proto::RobotCommand& cmd) {
    LowLevelRobotCommand llrc = createLowLevelRobotCommand(cmd, LastWorld);

    // check if the command is valid, otherwise don't send anything
    if(!validateRobotPacket(llrc)) {
        std::cout << "[processRobotCommand] LowLevelRobotCommand is not valid for our robots, no command is being sent!" << std::endl;
        printLowLevelRobotCommand(llrc);
        return;
    }

    robotTicks[cmd.id()]++;
    if (getMode() == utils::Mode::SERIAL) {
        sendSerialCommand(llrc);
    } else {
        sendGrSimCommand(cmd);
    }
}

/// send a serial command from a given robotcommand
void RobotHub::sendSerialCommand(LowLevelRobotCommand llrc) {

    // convert the LLRC to a bytestream which we can send
    std::shared_ptr<packed_protocol_message> bytestream = createRobotPacket(llrc);

    // Check if the message was created successfully
    if(!bytestream){
        std::cout << "[sendSerialCommand] The message was not created succesfully!" << std::endl;
        return;
    }
    packed_protocol_message packet = *bytestream;
    device->writeToDevice(packet);
    if (device->getMostRecentFeedback()) {
        publishRobotFeedback(createRobotFeedback(*device->getMostRecentFeedback()));
        device->removeMostRecentFeedback();
    }
}


/// send a GRSim command from a given robotcommand
void RobotHub::sendGrSimCommand(const roboteam_proto::RobotCommand& robotCommand) {
        this->grsimCommander->queueGRSimCommand(robotCommand);
}
void RobotHub::setMode() {
    if (target == "grsim")
        mode = utils::Mode::GRSIM;
    else if (target == "serial")
        mode = utils::Mode::SERIAL;
    else {
        std::cerr << "robot_output_target not set with ROS, will default to SERIAL" << std::endl;
        mode = utils::Mode::SERIAL;
    }
}

void RobotHub::publishRobotFeedback(LowLevelRobotFeedback llrf) {
    if (llrf.id >= 0 && llrf.id < 16) {
       // feedbackPublisher.publish(toRobotFeedback(llrf));
    }
}

} // robothub
} // rtt

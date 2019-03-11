
#include <ros/param.h>
#include "Application.h"
#include "SerialDeviceManager.h"
#include "GRSim.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_msgs/RobotFeedback.h"
#include "packing.h"

namespace rtt {
namespace robothub {

Application::Application() {
    subscribeToROSTopics();

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
        utils::Mode currentMode = rtt::robothub::utils::stringToMode(modeStr);
        ROS_INFO_STREAM("Current mode : " << utils::modeToString(currentMode));
        return currentMode;
    }
    return mode;
}

/// Get output device from the ROS parameter server
std::string Application::getSerialDevice() {
    std::string deviceName = "none";
    if(ros::param::has("output_device")) {
        ros::param::get("output_device", deviceName);
    }
    ROS_INFO_STREAM("[getSerialDevice] Looking for serial device: " << deviceName);
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
    subWorldState = n.subscribe("world_state", 1, &Application::processWorldState, this);
    subRobotCommands = n.subscribe("robotcommands", 1000, &Application::processRobotCommand, this);
}

/// get Tick rate from ROS parameter server
int Application::getTickRate() {
    int roleHz = 120;
    if(ros::param::has("grsim/role_iterations_per_second")) {
        ros::param::get("role_iterations_per_second", roleHz);
    }
    ROS_INFO_STREAM("Refresh rate: " << roleHz << "hz");
    return roleHz;
}

void Application::loop(){
    ros::Rate rate(getTickRate());

    std::chrono::high_resolution_clock::time_point lastStatistics = std::chrono::high_resolution_clock::now();

    int nTick = 0;
    packed_robot_feedback fb;
    int currIteration = 0;

    while (ros::ok()) {
        rate.sleep();
        ros::spinOnce();

        grsimCommander->setBatch(getBatchingVariable());

        auto timeNow = std::chrono::high_resolution_clock::now();
        auto timeDiff = timeNow-lastStatistics;

        if (std::chrono::duration_cast<std::chrono::milliseconds>(timeDiff).count()>1000) {

            lastStatistics = timeNow;

            ROS_INFO_STREAM("==========| " << currIteration++ << " |==========");

            if (mode== utils::Mode::SERIAL) {


//                // Get the percentage of acks and nacks
//                auto total = acks+nacks;
//                auto ackPercent = static_cast<int>((acks/(double) total)*100);
//                auto nackPercent = static_cast<int>((nacks/(double) total)*100);
//
//                if (total==0) {
//                    ackPercent = 0;
//                    nackPercent = 0;
//                }
//
////				ROS_INFO_STREAM("Current port: " << serial_file_path );
//                ROS_INFO_STREAM("Sent messages the past second: " << total);
//                ROS_INFO_STREAM("Capacity: " << std::floor(total/(8.0*60.0)) << "%");
//                ROS_INFO_STREAM("Acks    : " << acks << " (" << ackPercent << ")");
//                ROS_INFO_STREAM("Nacks   : " << nacks << " (" << nackPercent << ")");
//
//                acks = 0;
//                acks = 0;
//                nacks = 0;
//
//                std::stringstream ssStatus;
//                ssStatus << std::endl;

            }

            if (mode == utils::Mode::GRSIM) {
//                ROS_INFO_STREAM("Network messages sent: " << networkMsgs);
//                networkMsgs = 0;
//                if (grsimCommander.isBatch()) {
//                    ROS_INFO_STREAM("Batching : yes");
//                    rtt::GRSimCommander::Stats stats = grsimCommander.consumeStatistics();
//
//                    ROS_INFO_STREAM("Average efficiency:       " << std::floor(stats.averageEfficiency*100) << "%");
//                    ROS_INFO_STREAM("Number of forced flushes: " << stats.numForcedFlushes);
//                    ROS_INFO_STREAM("Current threshold:        " << stats.threshold);
//                }
//                else {
//                    ROS_INFO_STREAM("Batching : no");
//                }
            }
        }
    }
}


void Application::processWorldState(const roboteam_msgs::World& world){
    LastWorld = std::make_shared<roboteam_msgs::World>(world);
}

void Application::processRobotCommand(const roboteam_msgs::RobotCommand& cmd) {
    LowLevelRobotCommand llrc = createLowLevelRobotCommand(cmd, LastWorld);

    // check if the command is valid, otherwise don't send anything
    if(!validateRobotPacket(llrc)) {
        ROS_ERROR_STREAM("[processRobotCommand] LowLevelRobotCommand is not valid for our robots, no command is being sent to GrSim!");
        printLowLevelRobotCommand(llrc);
        return;
    }

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

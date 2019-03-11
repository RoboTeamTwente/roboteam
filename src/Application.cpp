//
// Created by mrlukasbos on 8-3-19.
//

#include <ros/param.h>
#include "Application.h"
#include "SerialDeviceManager.h"

namespace rtt {
namespace robothub {

Application::Application() {
    subscribeToROSTopics();

    if (getMode() == utils::Mode::SERIAL) {
        device = std::make_shared<SerialDeviceManager>(getSerialDevice());
    } else {

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

/// subscribe to ROS topics
void Application::subscribeToROSTopics(){
    subWorldState = n.subscribe("world_state", 1, &Application::processWorldState, this);
    subRobotCommands = n.subscribe("robotcommands", 1000, &Application::processRobotCommand, this);
}

void Application::loop(){

    // Get the number of iterations per second
    int roleHz = 120;
    ros::param::get("role_iterations_per_second", roleHz);
    ros::Rate loop_rate(roleHz);
    ROS_INFO_STREAM("Refresh rate: " << roleHz << "hz");

    std::chrono::high_resolution_clock::time_point lastStatistics = std::chrono::high_resolution_clock::now();

    int nTick = 0;

    rtt::packed_robot_feedback fb;

    int currIteration = 0;

    while (ros::ok()) {
        loop_rate.sleep();
        ros::spinOnce();

        grsimCmd.setBatch(grSimBatch());

        auto timeNow = std::chrono::high_resolution_clock::now();
        auto timeDiff = timeNow-lastStatistics;

        if (std::chrono::duration_cast<milliseconds>(timeDiff).count()>1000) {

            lastStatistics = timeNow;

            ROS_INFO_STREAM("==========| " << currIteration++ << " |==========");

            if (currentMode==Mode::SERIAL) {


                // Get the percentage of acks and nacks
                auto total = acks+nacks;
                auto ackPercent = static_cast<int>((acks/(double) total)*100);
                auto nackPercent = static_cast<int>((nacks/(double) total)*100);

                if (total==0) {
                    ackPercent = 0;
                    nackPercent = 0;
                }

//				ROS_INFO_STREAM("Current port: " << serial_file_path );
                ROS_INFO_STREAM("Sent messages the past second: " << total);
                ROS_INFO_STREAM("Capacity: " << std::floor(total/(8.0*60.0)) << "%");
                ROS_INFO_STREAM("Acks    : " << acks << " (" << ackPercent << ")");
                ROS_INFO_STREAM("Nacks   : " << nacks << " (" << nackPercent << ")");

                acks = 0;
                acks = 0;
                nacks = 0;

                std::stringstream ssStatus;
                ssStatus << std::endl;

            }

            if (currentMode==Mode::GRSIM) {
                ROS_INFO_STREAM("Network messages sent: " << networkMsgs);
                networkMsgs = 0;
                if (grsimCmd.isBatch()) {
                    ROS_INFO_STREAM("Batching : yes");
                    rtt::GRSimCommander::Stats stats = grsimCmd.consumeStatistics();

                    ROS_INFO_STREAM("Average efficiency:       " << std::floor(stats.averageEfficiency*100) << "%");
                    ROS_INFO_STREAM("Number of forced flushes: " << stats.numForcedFlushes);
                    ROS_INFO_STREAM("Current threshold:        " << stats.threshold);
                }
                else {
                    ROS_INFO_STREAM("Batching : no");
                }
            }
        }
}


void Application::processWorldState(const roboteam_msgs::World& world){
    LastWorld = std::make_shared<roboteam_msgs::World>(world);
}

void Application::processRobotCommand(const roboteam_msgs::RobotCommand& cmd) {
    sendCommand(cmd);
}


/// send a serial command from a given robotcommand
void Application::sendSerialCommand(const roboteam_msgs::RobotCommand& robotCommand) {

    // Turn roboteam_msgs:RobotCommand into a message
    boost::optional<rtt::packed_protocol_message> bytesOptional = rtt::createRobotPacket(robotCommand, LastWorld);

    // Check if the message was created successfully
    if(!bytesOptional){
        return SerialResultStatus::COMMAND_TO_PACKET_FAILED;
    }
    // Get the bytes from the message
    rtt::packed_protocol_message ppm = *bytesOptional;

    // Write the command to the robot
    SerialResultStatus writeStatus = writeRobotCommand(robotCommand);
    if(writeStatus != SerialResultStatus::SUCCESS){
        ROS_ERROR_STREAM("[processSerialCommand] Error while writing the command to the robot!");
        return writeStatus;
    }

    return readStatus;
}

/// send a GRSim command from a given robotcommand
void Application::sendGrSimCommand(const roboteam_msgs::RobotCommand& robotCommand) {
    rtt::LowLevelRobotCommand llrc = rtt::createLowLevelRobotCommand(robotCommand,LastWorld);
    if(rtt::validateRobotPacket(llrc)) {
        //send the command
        grsimCmd.queueGRSimCommand(command);
        grsimStatusMap[command.id].acks++;
        networkMsgs++;
    }
    else{
        ROS_ERROR_STREAM("LowLevelRobotCommand is not valid for our robots, no command is being sent to GrSim!");
        printLowLevelRobotCommand(llrc);
    }
}

}
}
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <chrono>
#include <ios>
#include <string>
#include <signal.h>
#include <bitset>

#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_msgs/RobotSerialStatus.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/RobotFeedback.h"

#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/grSim_Commands.pb.h"
#include "roboteam_utils/normalize.h"
#include "roboteam_utils/SlowParam.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64MultiArray.h"

#include "utilities.h"
#include "SerialDeviceManager.h"
#include "GRSim.h"
#include "packing.h"

namespace {

enum class SerialResultStatus {
	COMMAND_TO_PACKET_FAILED,
	CANT_OPEN_PORT,
	CONNECTION_CLOSED_BY_PEER,
	UNKNOWN_WRITE_ERROR,
	UNKNOWN_READ_ERROR,
	STRANGE_RESPONSE,
	ACK,
	NACK,
	SUCCESS,
};

enum class Mode {
	SERIAL,
	GRSIM,
	UNDEFINED
};

enum ACK_FLAGS {
	ACK         = 0b00000001,	// Ack received
	BATTERY     = 0b00000010,	// Robot battery low!
	BALL_SENSOR = 0b00000100	// Robot sees ball
};

ros::Subscriber subWorldState;
ros::Subscriber subRobotCommands;
ros::Publisher pubRobotFeedback;

std::shared_ptr<roboteam_msgs::World> LastWorld;

Mode currentMode = Mode::UNDEFINED;

rtt::GRSimCommander grsimCmd(true);
SlowParam<bool> grSimBatch("grsim/batching", true);

std::string serial_file_path_param = "none";
std::string serial_file_path = "No basestation selected!";
std::string serial_file_paths[3] = {
	"/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_00000000001A-if00",
	"/dev/serial/by-id/usb-STMicroelectronics_Basestation_078-if00",
	"/dev/serial/by-id/usb-STMicroelectronics_Basestation_080-if00",
};

int acks = 0;
int nacks = 0;
int networkMsgs = 0;

namespace hub = ::rtt::robothub;
namespace utils = ::rtt::robothub::utils;

/**
 * Takes a robot command, converts it to bytes, and sends it to the robot
 * @param robotCommand : The command to be sent to the robot
 * @returns SerialResultStatus, e.g. UNKNOWN_WRITE_ERROR, SUCCESS, etc
 */
SerialResultStatus writeRobotCommand(const roboteam_msgs::RobotCommand& robotCommand){

	// Turn roboteam_msgs:RobotCommand into a message
	boost::optional<rtt::packed_protocol_message> bytesOptional = rtt::createRobotPacket(robotCommand, LastWorld);

	// Check if the message was created successfully
	if(!bytesOptional){
		return SerialResultStatus::COMMAND_TO_PACKET_FAILED;
	}
	// Get the bytes from the message
	rtt::packed_protocol_message ppm = *bytesOptional;

    hub::SerialDeviceManager sdm;
    sdm.writeToDevice(ppm)
}

/**
 * Takes a robot commands, sends it to the robot, receives its feedback
 * @param robotCommand : The robot command to be sent to the robot
 * @returns SerialResultStatus, e.g. CANT_OPEN_PORT, WRITE_ERROR, ACK, etc
 */
SerialResultStatus processSerialCommand(const roboteam_msgs::RobotCommand& robotCommand) {

	// Check if the serial port is open
	if(!ensureSerialport()){
		ROS_ERROR("[processSerialCommand] Port not open. Can't send command");
		return SerialResultStatus::CANT_OPEN_PORT;
	}

	// Write the command to the robot
	SerialResultStatus writeStatus = writeRobotCommand(robotCommand);
	if(writeStatus != SerialResultStatus::SUCCESS){
		ROS_ERROR_STREAM("[processSerialCommand] Error while writing the command to the robot!");
		return writeStatus;
	}

	// Read the feedback from the robot
	rtt::packed_robot_feedback feedback;
//	SerialResultStatus readStatus = readPackedRobotFeedback(feedback);
	// SerialResultStatus readStatus = readBoringAck();
	SerialResultStatus readStatus = SerialResultStatus::ACK;

	if(readStatus != SerialResultStatus::ACK && readStatus != SerialResultStatus::NACK){
		ROS_ERROR_STREAM("Something went wrong while reading the feedback from the robot");
		return readStatus;
	}

	// Convert the rtt::packed_robot_feedback to LowLevelRobotFeedback
	rtt::LowLevelRobotFeedback llrf = rtt::createRobotFeedback(feedback);
	// Convert the LowLevelRobotFeedback to roboteam_msgs::RobotFeedback
	roboteam_msgs::RobotFeedback msg = rtt::toRobotFeedback(llrf);

	// Publish the RobotFeedback
	pubRobotFeedback.publish(msg);

//	rtt::printLowLevelRobotFeedback(llrf);
//	rtt::printRobotFeedback(msg);

	return readStatus;
}


void sendCommand(roboteam_msgs::RobotCommand command) {
	if(currentMode == Mode::GRSIM){
		rtt::LowLevelRobotCommand llrc= rtt::createLowLevelRobotCommand(command,LastWorld);
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
	}else
	if(currentMode == Mode::SERIAL){
		SerialResultStatus result = processSerialCommand(command);

		switch (result) {
			case SerialResultStatus::ACK:
				acks++;
				serialStatusMap[command.id].acks++;
				break;
			case SerialResultStatus::NACK:
				nacks++;
				serialStatusMap[command.id].nacks++;
				break;
			default:
				break;
		}
	}else
	if(currentMode == Mode::UNDEFINED){
		ROS_INFO_STREAM("[sendCommand] Mode undefined, not sending command");
	}

}

void processWorldState(const roboteam_msgs::World& world){
	LastWorld = std::make_shared<roboteam_msgs::World>(world);
}

void processRobotCommand(const roboteam_msgs::RobotCommand& cmd) {
	sendCommand(cmd);
}

} // anonymous namespace

int main(int argc, char *argv[]) {
    std::vector<std::string> args(argv, argv + argc);
    ros::init(argc, argv, "robothub");
    ros::NodeHandle n;

	// Get output device from the ROS parameter server
	if(ros::param::has("output_device")){
		ros::param::get("output_device", serial_file_path_param);
		if(serial_file_path_param != "none")
			ROS_INFO_STREAM("Output device given : " << serial_file_path_param);
	}

	// Get the mode of robothub, either "serial" or "grsim"
	std::string modeStr = "undefined";
	ros::param::get("robot_output_target", modeStr);
	currentMode = rtt::robothub::utils::stringToMode(modeStr);
	ROS_INFO_STREAM("Current mode : " << rtt::robothub::utils::modeToString(currentMode));


    // Get the number of iterations per second
    int roleHz = 120;
    ros::param::get("role_iterations_per_second", roleHz);
    ros::Rate loop_rate(roleHz);
	ROS_INFO_STREAM("Refresh rate: " << roleHz << "hz");

	std::chrono::high_resolution_clock::time_point lastStatistics = std::chrono::high_resolution_clock::now();

	int nTick = 0;

	rtt::packed_robot_feedback fb;

	subWorldState = n.subscribe("world_state", 1, processWorldState);
    subRobotCommands = n.subscribe("robotcommands", 1000, processRobotCommand);
	pubRobotFeedback = n.advertise<roboteam_msgs::RobotFeedback>("robotfeedback", 1000);

	// Check if the serial port is open
	if(currentMode == Mode::SERIAL && !ensureSerialport()){
		ROS_FATAL("Port not open. Can't communicate with the robots");
	}

	// Initialize the map that holds the serial status of all robots
	for(int i = 0; i < 16; i++){
		serialStatusMap[i] = roboteam_msgs::RobotSerialStatus();
	}

    int currIteration = 0;

    while (ros::ok()) {
        loop_rate.sleep();
        ros::spinOnce();



        grsimCmd.setBatch(grSimBatch());

        auto timeNow = std::chrono::high_resolution_clock::now();
        auto timeDiff = timeNow - lastStatistics;


        if (std::chrono::duration_cast<milliseconds>(timeDiff).count() > 1000) {

            lastStatistics = timeNow;

            ROS_INFO_STREAM("==========| " << currIteration++ << " |==========");


            if (currentMode == Mode::SERIAL) {

				// Check if the serial port is open
				if(!ensureSerialport()){
					ROS_FATAL("Port not open. Can't communicate with the robots");
					continue;
				}

                // Get the percentage of acks and nacks
                auto total = acks + nacks;
                auto ackPercent = static_cast<int>((acks / (double) total) * 100);
                auto nackPercent = static_cast<int>((nacks / (double) total) * 100);

                if (total == 0) {
                    ackPercent = 0;
                    nackPercent = 0;
                }

//				ROS_INFO_STREAM("Current port: " << serial_file_path );
				ROS_INFO_STREAM("Sent messages the past second: " << total );
				ROS_INFO_STREAM("Capacity: " << std::floor(total / (8.0 * 60.0)) << "%");
				ROS_INFO_STREAM("Acks    : " << acks << " (" << ackPercent << ")");
				ROS_INFO_STREAM("Nacks   : " << nacks << " (" << nackPercent << ")");

				acks=0;
				acks=0;
				nacks=0;

				std::stringstream ssStatus;
				ssStatus << std::endl;

				for (auto &pair : serialStatusMap) {
					int id = pair.first;
					roboteam_msgs::RobotSerialStatus& status = pair.second;

					status.id = id;
					status.end_timeframe = ros::Time::now();

					ssStatus << id;
					if(id < 10) ssStatus << " ";
					ssStatus << ": " << status.acks << "/" << (status.acks + status.nacks) << " \t";
					if((id+1) % 4 == 0){
						ssStatus << std::endl;
					}

					// Set the new start time.
					status.start_timeframe = ros::Time::now();
					// Reset the ack and nack counters.
					status.acks = 0;
					status.nacks = 0;
				}

				ROS_INFO_STREAM(ssStatus.str());

            }


            if (currentMode == Mode::GRSIM) {
                ROS_INFO_STREAM("Network messages sent: " << networkMsgs );
                networkMsgs = 0;
                if (grsimCmd.isBatch()) {
                    ROS_INFO_STREAM("Batching : yes");
                    rtt::GRSimCommander::Stats stats = grsimCmd.consumeStatistics();

                    ROS_INFO_STREAM("Average efficiency:       " << std::floor(stats.averageEfficiency * 100) << "%");
                    ROS_INFO_STREAM("Number of forced flushes: " << stats.numForcedFlushes);
                    ROS_INFO_STREAM("Current threshold:        " << stats.threshold);
                } else {
                    ROS_INFO_STREAM("Batching : no");
                }
            }
        }
    }

    return 0;

}
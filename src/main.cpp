#include <QtNetwork>
#include <boost/asio.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/algorithm/hex.hpp>

#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <chrono>
#include <ios>
#include <string>

namespace b = boost;

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

#include "roboteam_robothub/GRSim.h"

#include "roboteam_robothub/packing.h"

#define VERIFY_COMMANDS

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

// Publisher and subscriber to get robotcommands
ros::Subscriber subRobotCommands;
// Halt subscriber
ros::Subscriber subHalt;
// Publisher for the robot feedback
ros::Publisher pubRobotFeedback;
// Publisher for the serial status
ros::Publisher pubSerialStatus;

Mode currentMode = Mode::UNDEFINED;
bool halt = false;

void processHalt(const std_msgs::Bool::ConstPtr &msg) {
    ROS_INFO_STREAM("Received a halt: " << (int) msg->data );
    halt = msg->data;
}

//ros::Publisher pub;

rtt::GRSimCommander grsimCmd(true);
SlowParam<bool> grSimBatch("grsim/batching", true);






std::map<int, roboteam_msgs::RobotSerialStatus> serialStatusMap;
std::map<int, roboteam_msgs::RobotSerialStatus> grsimStatusMap;

std::string serial_file_path = "/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_00000000001A-if00";
//std::string serial_file_path = "/dev/ttyACM2";
//std::string serial_file_path = "/home/emiel/myfifo";

bool serialPortOpen = false;
boost::asio::io_service io;
boost::asio::serial_port serialPort(io);
SerialResultStatus prevResultStatus = SerialResultStatus::ACK;

int acks = 0;
int nacks = 0;
int networkMsgs = 0;




int char2int(char input){
	if(input >= '0' && input <= '9') return input - '0';
	if(input >= 'A' && input <= 'F') return input - 'A' + 10;
	if(input >= 'a' && input <= 'f') return input - 'a' + 10;
	throw std::invalid_argument("char2int : Invalid input string");
}

// This function assumes src to be a zero terminated sanitized string with an even number of [0-9a-f] characters, and target to be sufficiently large
void hex2bin(const char* src, rtt::packed_robot_feedback& target){
	int i;
	while(*src && src[1])
	{
		target.at(i) = char2int(*src)*16 + char2int(src[1]);
		src += 2;
		i++;
	}
}

std::string string_to_hex(const std::string& input){
	static const char* const lut = "0123456789ABCDEF";
	size_t len = input.length();

	std::string output;
	output.reserve(2 * len);
	for (size_t i = 0; i < len; ++i)
	{
		const unsigned char c = input[i];
		output.push_back(lut[c >> 4]);
		output.push_back(lut[c & 15]);
	}
	return output;
}


/**
 * Ensures that the serial port is open. Attempts to open the serial port if it is closed
 * @returns true if the port is open, false otherwise
 */
bool ensureSerialport(){
	// If the serial port is not open at the moment
	if (!serialPortOpen) {

		// Open the serial port
		ROS_INFO_STREAM("[ensureSerialport] Opening serial port " << serial_file_path << "...");
		boost::system::error_code ec;
		serialPort.open(serial_file_path, ec);

		// Check the status of the serial port
		switch (ec.value()) {

			case boost::system::errc::success:
				ROS_INFO("[ensureSerialport] Port opened");
				serialPortOpen = true;
				return true;

			default:
				ROS_ERROR_STREAM_THROTTLE(1, "[ensureSerialport] Error while opening serial port : (" << ec.value() << ") " << ec.message());
				return false;
		}
	}
	return true;
}

/**
 * Reads a 49-byte long hex string from the robot, drops the newline byte, converts it to bytes, and puts it in the 23-byte long byteArray.
 * @param byteArray - The array where the 23 bytes should be put in
 * @returns SerialResultStatus, e.g. ACK, NACK, UNKNOWN_READ_ERROR, etc
 */
SerialResultStatus readPackedRobotFeedback(rtt::packed_robot_feedback& byteArray){

	// Check if the serial port is open
	if(!ensureSerialport()){
		ROS_ERROR("[readPackedRobotFeedback] Port not open. Can't read feedback");
		return SerialResultStatus::CANT_OPEN_PORT;
	}

	// Create a buffer to receive the hex from the robot in
	boost::asio::streambuf hexBuffer;
	// Create an error_code object
	b::system::error_code ec;

	// Read bytes into hexBuffer until '\n' appears
	ROS_DEBUG("[readPackedRobotFeedback] Reading feedback...");
	size_t bytesReceived = boost::asio::read_until(serialPort, hexBuffer, '\n', ec);

	// Check if an error occured while reading
	if(ec){
		ROS_WARN_STREAM("[readPackedRobotFeedback] An error occured! " << ec.message());
		return SerialResultStatus::UNKNOWN_READ_ERROR;
	}
	// If there is an incorrect number of bytes
	if(bytesReceived != 49){
		ROS_WARN_STREAM("[readPackedRobotFeedback] Dropping message with incorrect size " << bytesReceived);
		return SerialResultStatus::STRANGE_RESPONSE;
	}

	// Commit the buffer. (emptying?)
	hexBuffer.commit(bytesReceived);
	// No idea
	std::istream is(&hexBuffer);
	// Create a string to move the hex in the buffer to
	std::string hexString;
	// Move the hex from the buffer to the string
	std::getline(is, hexString);
	// Replace \n with \0. Not used atm, but can't hurt
	hexString[bytesReceived-1] = '\0';
	// Convert the hex string to an uint8_t array. +2 to drop the ACK byte
	hex2bin(hexString.c_str()+2, byteArray);
	// Retrieve the byte that represents ACK or NACK
	int ackByte = char2int(hexString[0]) * 16 + char2int(hexString[1]);

	// === Print the values === //
//	std::cout << (ackByte ? "ACK" : "NACK") << " received " << bytesReceived << " bytes " << hexString << std::endl;
//	for(int i = 0; i < 48; i += 2){
//		printf("0x%c%c\t", hexString[i], hexString[i+1]);
//	}
//	printf("\n\t");
//	for(int i = 0; i < 23; i++){
//		printf("%i\t", byteArray.at(i));
//	}
//	printf("\n");
	// ======================== //

	if(ackByte)
		return SerialResultStatus::ACK;
	else{
		return SerialResultStatus::NACK;
	}
}

/**
 * Takes a robot command, converts it to bytes, and sends it to the robot
 * @param robotCommand : The command to be sent to the robot
 * @returns SerialResultStatus, e.g. UNKNOWN_WRITE_ERROR, SUCCESS, etc
 */
SerialResultStatus writeRobotCommand(const roboteam_msgs::RobotCommand& robotCommand){

	// Check if the serial port is open
	if(!ensureSerialport()){
		ROS_ERROR("[sendRobotCommand] Port not open. Can't write command");
		return SerialResultStatus::CANT_OPEN_PORT;
	}

	// Get the latest world state
	b::optional<roboteam_msgs::World> worldOptional;
	if (rtt::LastWorld::have_received_first_world()) {
		worldOptional = rtt::LastWorld::get();
	}

	// Turn roboteam_msgs:RobotCommand into a message
	boost::optional<rtt::packed_protocol_message> bytesOptional = rtt::createRobotPacket(robotCommand, worldOptional);

	// Check if the message was created successfully
	if(!bytesOptional){
		return SerialResultStatus::COMMAND_TO_PACKET_FAILED;
	}
	// Get the bytes from the message
	rtt::packed_protocol_message ppm = *bytesOptional;

	// Print the bytes in HEX
//	for (int i = 0; i < ppm.size(); i++) {
//		printf("%02x ", ppm.data()[i]);
//	}
//	printf("\n");

	// Error code
	b::system::error_code ec;
	// Write the bytes to the basestation
	ROS_DEBUG("[sendRobotCommand] Writing command...");
	b::asio::write(serialPort, boost::asio::buffer(ppm.data(), ppm.size()), ec);

	// Check the error_code
	switch (ec.value()) {
		case boost::asio::error::eof:
			ROS_ERROR_STREAM("[sendSerialCommands] Connection closed by peer while writing! Closing port and trying to reopen...");
			serialPort.close();
			serialPortOpen = false;
			return SerialResultStatus::CONNECTION_CLOSED_BY_PEER;

		case boost::system::errc::success:
			return SerialResultStatus::SUCCESS;

		default:
			ROS_ERROR_STREAM("[sendRobotCommand] Unknown write error : (" << ec.value() << ")! Closing the port and trying to reopen...");
			serialPort.close();
			serialPortOpen = false;
			return SerialResultStatus::UNKNOWN_WRITE_ERROR;
	}
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
	SerialResultStatus readStatus = readPackedRobotFeedback(feedback);

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






Mode stringToMode(const std::string& type) {
    if (type == "serial") {
        return Mode::SERIAL;
    } else if (type == "grsim") {
        return Mode::GRSIM;
    } else {
        return Mode::UNDEFINED;
    }
}
std::string modeToString(Mode mode) {
    switch(mode) {
        case Mode::SERIAL:
            return "serial";
        case Mode::GRSIM:
            return "grsim";
		case Mode::UNDEFINED:
			return "grsim";
    }
}

void sendCommand(roboteam_msgs::RobotCommand command) {

	if(currentMode == Mode::GRSIM){
		grsimCmd.queueGRSimCommand(command);
		grsimStatusMap[command.id].acks++;
		networkMsgs++;
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

void processRobotCommandWithIDCheck(const ros::MessageEvent<roboteam_msgs::RobotCommand>& msgEvent) {

    auto const & msg = *msgEvent.getMessage();

    auto pubName = msgEvent.getPublisherName();

    int robotLoc = pubName.find("robot");
    if (robotLoc != std::string::npos) {
        b::optional<int> robotID;

        try {
            auto robotIDLength2 = std::stoi(pubName.substr(robotLoc + 5, 2));
            robotID = robotIDLength2;
        } catch (...) {
            try {
                auto robotIDLength1 = std::stoi(pubName.substr(robotLoc + 5, 1));
                robotID = robotIDLength1;
            } catch (...) {
                // Neither matched; do nothing.
                ROS_ERROR("Neither matched!");
            }
        }

        if (robotID) {
            if (*robotID == msg.id) {
                sendCommand(msg);
            } else {
                ROS_ERROR_STREAM("Message sent by robot " << *robotID << " is not the same as ID in message which is " << msg.id << "!");
            }
        } else {
            // Parsing failed; let the message through, probably some test node.
            ROS_ERROR("Parsing failed!");
        }
    } else {
        // In this case the msg was sent by something else
        sendCommand(msg);
    }
}

} // anonymous namespace

int main(int argc, char *argv[]) {
    std::vector<std::string> args(argv, argv + argc);

    // ┌──────────────────┐
    // │  Initialization  │
    // └──────────────────┘

    // Create ROS node called robothub and subscribe to topic 'robotcommands'
    ros::init(argc, argv, "robothub");
    ros::NodeHandle n;

	// Get the mode of robothub, either "serial" or "grsim"
	std::string modeStr = "undefined";
	ros::param::get("robot_output_target", modeStr);
	currentMode = stringToMode(modeStr);
	if(currentMode == Mode::UNDEFINED){
		ROS_WARN_STREAM("Watch out! The current mode '" << modeStr << "' is UNDEFINED. No packets will be sent");
	}else{
		ROS_INFO_STREAM("Current mode : " << modeToString(currentMode));
	}

    // Get the number of iterations per second
    int roleHz = 120;
    ros::param::get("role_iterations_per_second", roleHz);
    ros::Rate loop_rate(roleHz);
	ROS_INFO_STREAM("Refresh rate: " << roleHz << "hz");

	using namespace std;
	using namespace std::chrono;

	high_resolution_clock::time_point lastStatistics = high_resolution_clock::now();

//	int nTick = 0;
//	while (ros::ok()) {
//		loop_rate.sleep();
//		ros::spinOnce();
//
//		auto timeNow = std::chrono::high_resolution_clock::now();
//		auto timeDiff = timeNow - lastStatistics;
//
//		// ┌──────────────────┐
//		// │   Every second   │
//		// └──────────────────┘
//		ROS_INFO_STREAM("\nTick " << ++nTick);
//		lastStatistics = timeNow;
//		readPackedRobotFeedback();
//	}
//	return 0;

    // Publisher and subscriber to get robotcommands
    subRobotCommands = n.subscribe("robotcommands", 1000, processRobotCommandWithIDCheck);
    // Halt subscriber
    subHalt = n.subscribe("halt", 1, processHalt);
	// Publisher for the robot feedback
	pubRobotFeedback = n.advertise<roboteam_msgs::RobotFeedback>("robotfeedback", 100);
	// Publisher for the serial status
    pubSerialStatus = n.advertise<roboteam_msgs::RobotSerialStatus>("robot_serial_status", 100);

	// Check if the serial port is open
	if(currentMode == Mode::SERIAL && !ensureSerialport()){
		ROS_FATAL("Port not open. Can't communicate with the robots");
	}


    // ┌──────────────────┐
    // │    Main loop     │
    // └──────────────────┘

    int currIteration = 0;

    while (ros::ok()) {
        loop_rate.sleep();
        ros::spinOnce();

        // Stop all robots if needed!
        if (halt) {
            roboteam_msgs::RobotCommand command;
            for (int i = 0; i < 16; ++i) {
                command.id = i;

                sendCommand(command);
            }
        }

        grsimCmd.setBatch(grSimBatch());

        auto timeNow = high_resolution_clock::now();
        auto timeDiff = timeNow - lastStatistics;

        // ┌──────────────────┐
        // │   Every second   │
        // └──────────────────┘
        if (duration_cast<milliseconds>(timeDiff).count() > 1000) {

            lastStatistics = timeNow;

            ROS_INFO_STREAM("==========| " << currIteration++ << " |==========");

            if (halt) {
                ROS_WARN_STREAM("Halting status : !!!HALTING!!!");
            } else {
                ROS_INFO_STREAM("Halting status : Not halting");
            }

            // ┌──────────────────┐
            // │      Serial      │
            // └──────────────────┘
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
				ROS_INFO_STREAM("Capacity: " << std::floor(total / 360.0) << "%");
				ROS_INFO_STREAM("Acks    : " << acks << " (" << ackPercent << ")");
				ROS_INFO_STREAM("Nacks   : " << nacks << " (" << nackPercent << ")");

				acks=0;
				nacks=0;

				/* Emiel : TODO Check if anything actually listens to roboteam_msgs::RobotSerialStatus
				// Publish any status info on the robots
				for (auto &pair : serialStatusMap) {
					int id = pair.first;
					roboteam_msgs::RobotSerialStatus &status = pair.second;

					if (status.start_timeframe == ros::Time(0)) {
						// This is a new status message.
						// It has been initialized with an id of 0.
						// So we need to set it correctly.
						status.id = id;
					} else {
						// This status message is ready to be sent.
						status.end_timeframe = ros::Time::now();

						pubSerialStatus.publish(status);
					}

					// Set the new start time.
					status.start_timeframe = ros::Time::now();
					// Reset the ack and nack counters.
					status.acks = 0;
					status.nacks = 0;
				}
				*/

            }
            // ┌──────────────────┐
            // │      grSim       │
            // └──────────────────┘
            if (currentMode == Mode::GRSIM) {

                ROS_INFO_STREAM("Network messages sent: " << networkMsgs );
                networkMsgs = 0;

				/* Emiel : TODO Check if anything actually listens to roboteam_msgs::RobotSerialStatus
                // Publish any status info on the robots
                for (auto &pair : grsimStatusMap) {
                    int id = pair.first;
                    roboteam_msgs::RobotSerialStatus &status = pair.second;

                    if (status.start_timeframe == ros::Time(0)) {
                        // This is a new status message.
                        // It has been initialized with an id of 0.
                        // So we need to set it correctly.
                        status.id = id;
                    } else {
                        // This status message is ready to be sent.
                        status.end_timeframe = ros::Time::now();

                        pubSerialStatus.publish(status);
                    }

                    // Set the new start time.
                    status.start_timeframe = ros::Time::now();
                    // Reset the ack and nack counters.
                    status.acks = 0;
                    status.nacks = 0;
                }
				*/

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






// SlowParam<std::string> colorParam("our_color");
// SlowParam<std::string> grsim_ip("grsim/ip", "127.0.0.1");
// SlowParam<int>         grsim_port("grsim/port", 20011);

// Algorithm for efficient GRSim'ing:
// - Receive msg
// - Put in buffer
//
// When is the buffer flushed:
// - After every ros::spinOnce(), flush buffer
//      - Probably the cleanest, simplest. Not sure about effectiveness.
//      - If in one spinonce you receive one robot instruction twice
//        you drop a packet
//      - If it takes multiple cycles to get a packet from everyone then this
//        is a bad solution since it doesn't compress much.
//      - If we run at the same speed as a rolenode however it seems appropriate
//      - Maybe with an escape as soon as it drops one packet?
// - Every 6 messages
//      - Might drop a packet every once in a while
//      - Like this the most atm.
//      - What if there are 2 robots? Need to detect how many there are.
//      - Reset that every 0.25 seconds?
// - Every a certain amount of time
//      - Might drop a packet every once in a while as well
// - When a packet from every robot is received
//      - Needs to be reset every 0.25 seconds or smth to account for robots leaving e.d.
//      - If one rolenode crashes all robots have no instructions for 0.25 secs
//
// - Did some tests. With 4 testx's running it seems quite nicely separated, i.e.
//   0 1 2 3 4 0 1 2 3 4, for some permutation of 0 1 2 3 4.
//   So the following seems a nice approach. Also, the safety thing will make
//   sure that for bad permutations or nodes that run faster than others it will
//   still send packets at a better than worst-case-scenario rate. Did some more
//   tests with randomly started testx's, still works like a charm.
//
// - Every time the buffer has reached the threshold, it will flush.
// - Every 0.25 (or some other amount) of seconds robothub will check
//   how many robots it has seen it that time period
// - That will be its new threshold
// - As long as all rolenodes run at the same Hz it should work fine
// - If one crashes the system recovers in 0.25 secs.
// - Safety: if two (or more/less?) packets are dropped from a specific robot
//   robothub immediately evaluates how many robots there are, changes its
//   threshold, and flushes the buffer.
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
#include <signal.h>
#include <bitset>

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

enum ACK_FLAGS {
	ACK         = 0b00000001,	// Ack received
	BATTERY     = 0b00000010,	// Robot battery low!
	BALL_SENSOR = 0b00000100	// Robot sees ball
};

// Subscriber to get LastWorld
ros::Subscriber subWorldState;
// Publisher and subscriber to get robotcommands
ros::Subscriber subRobotCommands;
// Halt subscriber
ros::Subscriber subHalt;
// Publisher for the robot feedback
ros::Publisher pubRobotFeedback;
// Publisher for the serial status
ros::Publisher pubSerialStatus;

boost::optional<roboteam_msgs::World> LastWorld;

Mode currentMode = Mode::UNDEFINED;
bool halt = false;

rtt::GRSimCommander grsimCmd(true);
SlowParam<bool> grSimBatch("grsim/batching", true);

std::map<int, roboteam_msgs::RobotSerialStatus> serialStatusMap;
std::map<int, roboteam_msgs::RobotSerialStatus> grsimStatusMap;

bool serialPortOpen = false;

std::string serial_file_path_param = "none";
std::string serial_file_path = "No basestation selected!";
std::string serial_file_paths[3] = {
	"/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_00000000001A-if00",
	"/dev/serial/by-id/usb-STMicroelectronics_Basestation_078-if00",
	"/dev/serial/by-id/usb-STMicroelectronics_Basestation_080-if00",
};

// "The core object of boost::asio is io_service. This object is like the brain and the heart of the library."
// create the I/O service that talks to the serial device
boost::asio::io_service io;
boost::asio::serial_port serialPort(io);

int acks = 0;
int nacks = 0;
int networkMsgs = 0;

void processHalt(const std_msgs::Bool::ConstPtr &msg) {
	ROS_INFO_STREAM("Received a halt: " << (int) msg->data );
	halt = msg->data;
}

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
// Possible useful documentation
//		http://www.college-code.com/blog/wp-content/uploads/2008/11/boost_serial_port_demo.cpp
bool ensureSerialport(){
	// If the serial port is not open at the moment
	if (!serialPortOpen) {


		// If output device given
		if(serial_file_path_param != "none"){
			// Open the serial port
			ROS_INFO_STREAM("[ensureSerialport] Trying to open serial port " << serial_file_path_param << "...");
			boost::system::error_code ec;
			serialPort.open(serial_file_path_param, ec);

			// Check the status of the serial port
			switch (ec.value()) {
				// Port opened
				case boost::system::errc::success:
					ROS_INFO("[ensureSerialport] Port opened");
					serialPortOpen = true;
					serial_file_path = serial_file_path_param;
					return true;
				// Port not opened
				default:
					ROS_WARN_STREAM("[ensureSerialport] Error while opening serial port : (" << ec.value() << ") " << ec.message());
			}
		}else {
			for (std::string path : serial_file_paths) {
				// Open the serial port
				ROS_INFO_STREAM("[ensureSerialport] Trying to open serial port " << path << "...");
				boost::system::error_code ec;
				serialPort.open(path, ec);

				// Check the status of the serial port
				switch (ec.value()) {
					// Port opened
					case boost::system::errc::success:
						ROS_INFO("[ensureSerialport] Port opened");
						serialPortOpen = true;
						serial_file_path = path;
						return true;

					// Port not opened
					default:
						ROS_DEBUG_STREAM("[ensureSerialport] Error while opening serial port : (" << ec.value() << ") " << ec.message());
				}
			}
		}

		// No port could be opened
		ROS_ERROR_STREAM("[ensureSerialPort] No serial port could be opened!");
		return false;
	}
	// Port is already open
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
	ROS_DEBUG("[readPackedRobotFeedback] Feedback read");

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

SerialResultStatus readBoringAck(){
	// Check if the serial port is open
	if(!ensureSerialport()){
		ROS_ERROR("[readBoringAck] Port not open. Can't read feedback");
		return SerialResultStatus::CANT_OPEN_PORT;
	}

	// Create a buffer to receive the ack in
	boost::asio::streambuf ackBuffer;
	// Create an error_code object
	b::system::error_code ec;

	// Read bytes into buffer until '\n' appears
	ROS_DEBUG("[readBoringAck] Reading ack...");
	size_t bytesReceived = boost::asio::read_until(serialPort, ackBuffer, '\n', ec);
	ROS_DEBUG("[readBoringAck] Ack read");

	// Check if an error occured while reading
	if(ec){
		ROS_WARN_STREAM("[readBoringAck] An error occured! " << ec.message());
		return SerialResultStatus::UNKNOWN_READ_ERROR;
	}
	// If there is an incorrect number of bytes
	if(bytesReceived != 2){
		ROS_WARN_STREAM("[readBoringAck] Dropping message with incorrect size " << bytesReceived);
		return SerialResultStatus::STRANGE_RESPONSE;
	}

	// Commit the buffer. (emptying?)
	ackBuffer.commit(bytesReceived);
	// No idea
	std::istream is(&ackBuffer);
	// Create a string to move the hex in the buffer to
	std::string ackString;
	// Move the hex from the buffer to the string
	std::getline(is, ackString);
	// Replace \n with \0. Not used atm, but can't hurt
	ackString[bytesReceived-1] = '\0';

	char ackByte = ackString[0];
//	std::bitset<8> x(ackByte);
//	std::cout << x << std::endl;


	// TODO Do something with the battery flag
	if(ackByte & ACK_FLAGS::BATTERY){
		ROS_WARN_STREAM_THROTTLE(1, "Low battery detected!");
	}

	// TODO Do something with the ball sensor flag
	if(ackByte & ACK_FLAGS::BALL_SENSOR){

	}

	if(ackByte & ACK_FLAGS::ACK)
		return SerialResultStatus::ACK;
	else
		return SerialResultStatus::NACK;



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

}

    void printbits(rtt::packed_protocol_message byteArr){
        for(int b = 0; b < 12; b++){

            std::cout << "    ";
            uint8_t byte = byteArr[b];
//            for (int i = 0; i < 8; i++) {
            for (int i = 7; i >= 0; i--) {
                if ((byte & (1 << i)) == (1 << i)) {
                    std::cout << "1";
                } else {
                    std::cout << "0";
                }
            }
            std::cout << std::endl;
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

//	// Get the latest world state
//	b::optional<roboteam_msgs::World> worldOptional;
//	if (rtt::LastWorld::have_received_first_world()) {
//		worldOptional = rtt::LastWorld::get();
//	}

	// Turn roboteam_msgs:RobotCommand into a message
	boost::optional<rtt::packed_protocol_message> bytesOptional = rtt::createRobotPacket(robotCommand, LastWorld);

	// Check if the message was created successfully
	if(!bytesOptional){
		return SerialResultStatus::COMMAND_TO_PACKET_FAILED;
	}
	// Get the bytes from the message
	rtt::packed_protocol_message ppm = *bytesOptional;

	// printbits(*bytesOptional);

	// Print the bytes in HEX
	// for (int i = 0; i < ppm.size(); i++) {
	// 	printf("%02x ", ppm.data()[i]);
	// }
	// printf("\n");

	// Error code
	b::system::error_code ec;
	// Write the bytes to the basestation
	ROS_DEBUG("[sendRobotCommand] Writing command...");
	b::asio::write(serialPort, boost::asio::buffer(ppm.data(), ppm.size()), ec);
	ROS_DEBUG("[readPackedRobotFeedback] Command written");

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
		rtt::LowLevelRobotCommand llrc= rtt::createLowLevelRobotCommand(command,LastWorld);
		if(rtt::validateRobotPacket(llrc)) {
			//send the command
			grsimCmd.queueGRSimCommand(command);
			grsimStatusMap[command.id].acks++;
			networkMsgs++;
		}
		else{
			ROS_WARN_STREAM("LowLevelRobotCommand is not valid for our robots, no command is being sent to GrSim!");
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
	LastWorld = world;
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
            ROS_ERROR("Parsing failed!");
        }
    } else {
        // In this case the msg was sent by something else
        sendCommand(msg);
    }
}

} // anonymous namespace

void stressTest(){

	if(!ensureSerialport())
		return;

	const int id = 7;
	std::cout << "Running stress test on robot " << id << " ..." << std::endl;

	roboteam_msgs::RobotCommand cmd;
	cmd.id = id;
	rtt::packed_robot_feedback fb;

	int counter = 0;
	int acks = 0;

	auto timeStart = std::chrono::high_resolution_clock::now();

	while(counter < 100000){
		writeRobotCommand(cmd);

//		SerialResultStatus s = readPackedRobotFeedback(fb);
		SerialResultStatus s = readBoringAck();

		if(s == SerialResultStatus::ACK)
			acks++;
		counter++;

		if(counter % 10000 == 0){
			std::cout << counter << "..." << std::endl;
			fflush(stdout);
		}
	}

	auto timeEnd = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsed_seconds = timeEnd - timeStart;
	auto x = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed_seconds);

	std::cout << "Test complete!" << std::endl;
	std::cout << "    Duration : " << x.count() << "ms, Hz : " << ((1000*counter)/(double)x.count()) << std::endl;
	std::cout << "    Total    : " << counter << ", acks : " << acks << ", % : " << (100 * ((double)acks/(double)counter)) << std::endl;

}

void mySigintHandler(int sig){
	std::cout << "Shutting down...(" << sig << ")" << std::endl;
	serialPort.close();

	std::cout << "    Closing serial port..." << std::endl;
	serialPort.close();
	while(serialPort.is_open());

	std::cout << "    Stopping io service..." << std::endl;
	io.stop();
	while(!io.stopped());

	std::cout << "    Shutting down ros node..." << std::endl;
	ros::shutdown();
	while(ros::ok());
	std::cout << "Shutdown complete" << std::endl;
}

int main(int argc, char *argv[]) {
    std::vector<std::string> args(argv, argv + argc);

    // ┌──────────────────┐
    // │  Initialization  │
    // └──────────────────┘

//		stressTest(); return 0;

	// Create ROS node called robothub and subscribe to topic 'robotcommands'
    ros::init(argc, argv, "robothub", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;

	// Bind ctrl+c to custom signal handler
	// http://wiki.ros.org/roscpp/Overview/Initialization%20and%20Shutdown
	signal(SIGINT, mySigintHandler);

	// Get output device
	if(ros::param::has("output_device")){
		ros::param::get("output_device", serial_file_path_param);
		if(serial_file_path_param != "none")
			ROS_INFO_STREAM("Output device given : " << serial_file_path_param);
	}

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

	int nTick = 0;

	rtt::packed_robot_feedback fb;

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
//		readPackedRobotFeedback(fb);
//	}
//	return 0;

	// Subscriber to get LastWorld
	subWorldState = n.subscribe("world_state", 1, processWorldState);

    // Publisher and subscriber to get robotcommands
    subRobotCommands = n.subscribe("robotcommands", 1000, processRobotCommandWithIDCheck);
    // Halt subscriber
    subHalt = n.subscribe("halt", 1, processHalt);
	// Publisher for the robot feedback
	pubRobotFeedback = n.advertise<roboteam_msgs::RobotFeedback>("robotfeedback", 1000);
	// Publisher for the serial status
    pubSerialStatus = n.advertise<roboteam_msgs::RobotSerialStatus>("robot_serial_status", 1000);

	// Check if the serial port is open
	if(currentMode == Mode::SERIAL && !ensureSerialport()){
		ROS_FATAL("Port not open. Can't communicate with the robots");
	}

	// Initialize the map that holds the serial status of all robots
	for(int i = 0; i < 16; i++){
		serialStatusMap[i] = roboteam_msgs::RobotSerialStatus();
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
				command.use_angle = false;
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
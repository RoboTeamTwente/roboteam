#include <boost/asio.hpp>
#include <boost/format.hpp>
namespace bf = boost;
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_robothub/packing.h"

namespace rtt {

template<
    typename T
>
std::string get_pretty_value(T val) {
    return std::to_string(val);
}

template<>
std::string get_pretty_value<bool>(bool val) {
    if (val) {
        return "true";
    } else {
        return "false";
    }
}

namespace {

bool portOpened = false;

std::string get_safe_input(std::string question = "", std::string defaultValue = "") {
    if (!question.empty()) {
        std::cout << question;
    }

    std::string input_str;
    
    if(std::getline(std::cin, input_str)) {
        if (input_str.empty()) {
            return defaultValue;
        }

        return input_str;
    }

    std::cout << "\nIO error. Aborting.\n";
    exit(1);
}

// http://www.boost.org/doc/libs/1_40_0/doc/html/boost_asio/overview/serial_ports.html

boost::asio::io_service io;
boost::asio::serial_port serialPort(io);

// All possible standard baud rates
std::vector<std::string> baudRates = {
    "110",
    "300",
    "600",
    "1200",
    "2400",
    "4800",
    "9600",
    "14400",
    "19200",
    "28800",
    "38400",
    "56000",
    "57600",
    "115200"
};

// "Extended" standard baud rates
std::vector<std::string> nonStandardBaudRates = {
    "128000",
    "153600",
    "230400",
    "256000",
    "460800",
    "921600 ",
};

void testMessage() {
    auto getFloat = [](std::string name, float def) {
        return std::stof(get_safe_input(name, std::to_string(def)));
    };
    auto getBool = [](std::string name, bool def) {
        return get_safe_input(name, get_pretty_value(def)) == "true";
    };

    int id              = std::stoi( // Whoops
                     get_safe_input("int   id             (       default 7)    : ", "7"));
    bool active         = getBool  ("bool  active         (       default true) : ", true);
    float x_vel         = getFloat ("float x_vel          (m/s,   default 0)    : ", 0);
    float y_vel         = getFloat ("float y_vel          (m/s,   default 0)    : ", 0);
    float w             = getFloat ("float w              (rad/s, default 0)    : ", 0);
    bool dribbler       = getBool  ("bool  dribbler       (       default false): ", false);
    bool kicker         = getBool  ("bool  kicker         (       default false): ", false);
    bool kicker_forced  = getBool  ("bool  kicker_forced  (       default false): ", false);
    float kicker_vel    = getFloat ("float kicker_vel     (m/s,   default 0)    : ", 0);
    bool chipper        = getBool  ("bool  chipper        (       default false): ", false);
    bool chipper_forced = getBool  ("bool  chipper_forced (       default false): ", false);
    float chipper_vel   = getFloat ("float chipper_vel    (m/s,   default 0)    : ", 0);

    roboteam_msgs::RobotCommand command;
    command.id = id;
    command.active = active;
    command.x_vel = x_vel;
    command.y_vel = y_vel;
    command.w = w;
    command.dribbler = dribbler;
    command.kicker = kicker;
    command.kicker_forced = kicker_forced;
    command.kicker_vel = kicker_vel;
    command.chipper = chipper;
    command.chipper_forced = chipper_forced;
    command.chipper_vel = chipper_vel;

    std::cout << "\n";

    auto llcommand = createLowLevelRobotCommand(command);

    std::cout << "Low level robot command after conversion from robot command:\n";
    std::cout << "id:                 " << llcommand.id << "\n";
    std::cout << "robot_vel:          " << llcommand.robot_vel << " mm/s\n";
    std::cout << "ang:                " << llcommand.ang << " (where 512 = 2 * PI)\n";
    std::cout << "rot_cclockwise:     " << llcommand.rot_cclockwise << "\n";
    std::cout << "w:                  " << llcommand.w << " deg/s\n";
    std::cout << "punt_power:         " << std::to_string(llcommand.punt_power) << " (where 255 = max power)\n";
    std::cout << "do_kick:            " << llcommand.do_kick << "\n";
    std::cout << "do_chip:            " << llcommand.do_chip << "\n";
    std::cout << "forced:             " << llcommand.forced << "\n";
    std::cout << "dribble_cclockwise: " << llcommand.dribble_cclockwise << "\n";
    std::cout << "dribble_vel:        " << std::to_string(llcommand.dribble_vel) << " (where 7 = max dribbler speed)\n";
    
    std::cout << "\n";

    if (auto possibleRobotPacket = createRobotPacket(command)) {
        auto msg = *possibleRobotPacket;

        std::cout << "\n";
        std::cout << "Binary packet:\n";

        for (const auto& byte : msg) {
            std::cout << "\t" << byteToBinary(byte) << "\t" << std::to_string(byte) << "\n";
        }

    } else {
        std::cout << "Somehow a packet could not be created. See the packet definition "
                  << "in packing.cpp and compare the bounds for values to the values of "
                  << "the low level robot command.\n";
    };
}

/// @brief Different ways a serial port may be flushed.
enum flush_type
{
  flush_receive = TCIFLUSH,
  flush_send = TCIOFLUSH,
  flush_both = TCIOFLUSH
};

/// @brief Flush a serial port's buffers.
///
/// @param serial_port Port to flush.
/// @param what Determines the buffers to flush.
/// @param error Set to indicate what error occurred, if any.
void flush_serial_port(
    boost::asio::serial_port& serial_port,
    flush_type what,
    boost::system::error_code& error
    ) {
    if (0 == ::tcflush(serial_port.lowest_layer().native_handle(), what)) {
        error = boost::system::error_code();
    } else {
        error = boost::system::error_code(errno, boost::asio::error::get_system_category());
    }
}

} // anonymous namespace

int main(const std::vector<std::string>& arguments) {
    ////////////////////////
    // Checking arguments //
    ////////////////////////


    
    if (arguments.size() < 2) {
        std::cout << "Too few arguments, please specify output file and robotID. Aborting.\n";
        return 1;
    }

    if (arguments.at(0) == "msg") {
        testMessage();
        exit(0);
    }

    if (std::find(arguments.begin(), arguments.end(), "--help") != arguments.end()) {
        std::cout << "Arguments: either a port (/dev/ttyACM0 or something), --help for this, or msg to test the roboteam_msg to robot packet mechanism.\n";
        exit(0);
    }

    std::string output_file = arguments.at(0);

    int robotID = std::stoi(arguments.at(1));

    std::cout << "Output file: " << output_file << " robotID: " << robotID << "\n";

    /////////////////////
    // Creating packet //
    /////////////////////


    


    



    auto packetType = get_safe_input("Example packet or initialize packet by hand (EXAMPLE/manual/ forward (f)/ backward (b) / left (l) /right (r)/ stop (s)? ", "EXAMPLE");

    // TODO: w should be ang, w_vel should be w
    int id;
    int robot_vel;
    int ang;
    bool rot_cclockwise;
    int w;
    uint8_t kick_force;
    bool do_kick;
    bool chip;
    bool forced;
    bool dribble_cclockwise;
    uint8_t dribble_vel;

    if (packetType == "manual") {
        std::cout << "Sending a manual packet.\n";

        id                 = std::stoi(get_safe_input("id (0-15, 7): ", "7"));
        robot_vel          = std::stoi(get_safe_input("robot_vel (0-8191, 2000): ", "2000"));
        ang                = std::stoi(get_safe_input("ang (0-511, 300): ", "300"));
        rot_cclockwise     = get_safe_input("rot_cclockwise (true/false): ") == "true";
        w                  = std::stoi(get_safe_input("w (0-2047, 1000): ", "1000"));
        kick_force         = std::stoi(get_safe_input("kick_force (0-255, 200): ", "200"));
        do_kick            = get_safe_input("do_kick (true/false): ") == "true";
        chip               = get_safe_input("chip (true/false): ") == "true";
        forced             = get_safe_input("forced (true/false): ") == "true";
        dribble_cclockwise = get_safe_input("dribble_cclockwise (true/false): ") == "true";
        dribble_vel        = std::stoi(get_safe_input("dribble_vel (0-7, 5): ", "5"));
    } else if (packetType == "EXAMPLE") {
        std::cout << "Sending an example packet.\n";
        
        id = 7;
        robot_vel = 2000;
        ang = 300;
        rot_cclockwise = true;
        w = 1000;
        kick_force = 200;
        do_kick = true;
        chip = false;
        forced = true;
        dribble_cclockwise = true;
        dribble_vel = 5;
    } else if (packetType == "f" || packetType == "b" || packetType == "l" || packetType == "r" || 
    		   packetType == "fl" || packetType == "fr" || packetType == "bl" || packetType == "br" ) {
        id = robotID;
        robot_vel = 2000;
        ang = 0;
        rot_cclockwise = false;
        w = 0;
        if (packetType == "f") {
            rot_cclockwise = false;
            w = 10;
        } else if (packetType == "b") {
            rot_cclockwise = true;
            w = 10;
        }
        
        
        kick_force = 0;
        do_kick = false;
        chip = false;
        forced = false;
        dribble_cclockwise = false;
        dribble_vel = 0;

        if (packetType == "f") {
            ang = 128;
        } else if (packetType == "b") {
            ang = 384;
        } else if (packetType == "l") {
            ang = 256;
        } else if (packetType == "r") {
            ang = 0;
        } else if (packetType == "fl") {
            ang = 192;
        } else if (packetType == "fr") {
            ang = 64;
        } else if (packetType == "bl") {
            ang = 320;
        } else if (packetType == "br") {
            ang = 448;
        }

    } else if (packetType == "s") {
    	id = robotID;
        robot_vel = 0;
        ang = 0;
        rot_cclockwise = true;
        w = 0;
        kick_force = 200;
        do_kick = true;
        chip = false;
        forced = true;
        dribble_cclockwise = true;
        dribble_vel = 5;
    } 


    /////////////////////
    // Printing packet //
    /////////////////////

    // #define FIELD(id) std::cout << bf::format("\t%-20s %-20s\n") % #id % get_pretty_value(id);

    // std::cout << "Packet:\n";
    // FIELD(id);
    // FIELD(robot_vel);
    // FIELD(ang);
    // FIELD(rot_cclockwise);
    // FIELD(w);
    // FIELD(kick_force);
    // FIELD(do_kick);
    // FIELD(chip);
    // FIELD(forced);
    // FIELD(dribble_cclockwise);
    // FIELD(dribble_vel);

    std::cout << "Creating message... ";

    ///////////////////////
    // Converting packet //
    ///////////////////////

    auto possibleMsg = createRobotPacket(
            id,
            robot_vel,
            ang,
            rot_cclockwise,
            w,
            kick_force,
            do_kick,
            chip,
            forced,
            dribble_cclockwise,
            dribble_vel
            );

    if (!possibleMsg) {
        std::cout << "An error occurred while creating the message. Please look at the constraints of createRobotPacket(). Aborting.";
        return 1;
    }

    // std::cout << "All params ok.\n";

    ////////////////////////////
    // Printing binary packet //
    ////////////////////////////

    // std::cout << "Message contents: \n";

    auto msg = *possibleMsg;

    // for (const auto& byte : msg) {
    //     std::cout << "\t" << byteToBinary(byte) << "\t" << std::to_string(byte) << "\n";
    // }

    //////////////////////////
    // Creating serial port //
    //////////////////////////

    // if (!get_safe_input("Press enter to open the port or type something to cancel...").empty()) {
    //     std::cout << "Sending message canceled. Aborting.\n";
    //     return 0;
    // }

    // std::cout << "Creating serial port... ";


    if (!portOpened) {

    	boost::system::error_code errorCode;
	    std::cout << "Opening serial port \n";
        serialPort.open(output_file, errorCode);
	    switch (errorCode.value()) {
	        case boost::system::errc::success:
	            // Great!
	            break;
	        default:
	            std::cout << "An error occurred while creating the file object. Have you passed the right path? "
	                      << "Boost error: " << errorCode << ", message: " << errorCode.message() << "\n";
	            exit(1);
	            break;
	    }

	    portOpened = true;

    }
    

    // std::cout << "Done.\n";

    ///////////////////////
    // Setting baud rate //
    ///////////////////////

    boost::asio::serial_port_base::baud_rate baudRate;
    serialPort.get_option(baudRate);
    // std::cout << "Baud rate: " << baudRate.value() << "\n";
    
    // std::cout << "Other standard baud rates: \n";
    // for (auto br : baudRates) {
    //     std::cout << "    " << br << "\n";
    // }
    // std::cout << "Less standard baud rates: \n";
    // for (auto br : nonStandardBaudRates) {
    //     std::cout << "    " << br << "\n";
    // }

    // auto userBaudRateStr = get_safe_input("Set baud rate (or empty to leave at standard): ");
    // if (!userBaudRateStr.empty()) {
    //     try {
    //         int userBaudRate = std::stoi(userBaudRateStr);
    //         baudRate = boost::asio::serial_port_base::baud_rate(userBaudRate);
    //         boost::system::error_code ec;
    //         serialPort.set_option(baudRate, ec);

    //         if (ec != boost::system::errc::success) {
    //             std::cout << "Could not set baud rate. Error: " << ec << ". " << ec.message() << "\n";
    //         }

    //         serialPort.get_option(baudRate);
    //         std::cout << "Final baud rate: " << baudRate.value() << "\n";
    //     } catch (std::invalid_argument const & ex) {
    //         std::cout << "Invalid baudRate specified. Continuing with baudrate " << baudRate.value() << "\n";
    //     } catch (std::out_of_range const & ex) {
    //         std::cout << "Specified baud rate too large for an int. Continuing with baudrate " << baudRate.value() << "\n";
    //     }
    // }

    bool benchMark = get_safe_input("Benchmark (y/N)? ", "N") != "N";
    // bool benchMark = false;

    
    bool quickTest = get_safe_input("Quick test? (y/N) ", "N") != "N";

    if (benchMark) {
        ///////////////
        // Benchmark //
        ///////////////

        bool checkForAck = get_safe_input("Check for ACK (Y/n): ", "Y") != "n";
        int const MESSAGE_QUANTITY = std::stoi(get_safe_input("Amount of messages (10 000): ", "10000"));

        int const numBytes = 3;
        uint8_t ackCode[numBytes];

        if (!get_safe_input("Press enter to continue type something to cancel...").empty()) {
            std::cout << "Sending message canceled. Aborting.\n";
            return 0;
        }
        
        std::cout << "Sending packets..." << std::flush;

        // Communicatie todo
        // TODO: Make packet length 7 in hans's stuff
        // TODO: Make library of hans nrf stuff
        // gedaan: serial baud testing
        // gedaan: SPI baud testing
        // TODO: Make main pcb response 2 bytes instead of 2 ascii characters
        // gedaan: Optization for hans' code, useful or not? Nee
        // gedaan: WriteData optimaliseren bij Hans? Semi-lelijk

        // There are other clocks, but this is usually the one you want.
        // It corresponds to CLOCK_MONOTONIC at the syscall level.
        using Clock = std::chrono::steady_clock;
        using std::chrono::time_point;
        using std::chrono::duration_cast;
        using std::chrono::milliseconds;
        using namespace std::literals::chrono_literals;
        using std::this_thread::sleep_for;

        time_point<Clock> start = Clock::now();

        int failCount = 0;

        for (int i = 1; i < MESSAGE_QUANTITY + 1; ++i) {
            serialPort.write_some(boost::asio::buffer(msg.data(), msg.size()));

            // TODO: @Hack base station crutches! Pakcet length should be smaller
            serialPort.write_some(boost::asio::buffer(msg.data(), 1));

            // boost::system::error_code err;
            // flush_serial_port(serialPort, flush_type::flush_send, err);

            // if (err != boost::system::errc::success) {
                // std::cout << "Error flushing! Code: " << err.value() << ", message: " << err.message() << "\n";
            // }

            if (checkForAck) {
                serialPort.read_some(boost::asio::buffer(ackCode, numBytes - 1));

                ackCode[numBytes - 1] = 0;

                std::string returnMessage((char*) &ackCode[0]);

                if (returnMessage[1] == '0') {
                    failCount++;
                }
            }

            if (i % 100 == 0) {
                std::cout << "." << std::flush;
            }
        }

        time_point<Clock> end = Clock::now();

        milliseconds diff = duration_cast<milliseconds>(end - start);

        std::cout << "\nBenchmark done!\n";
        std::cout << "Sent " << MESSAGE_QUANTITY << " messages.\n";
        std::cout << "Duration: " << diff.count() << "ms" << std::endl;
        std::cout << "Failures: " << failCount << "\n";
    } else if (quickTest) {
        std::cout << "Quicktest!\n";

        id = robotID;
        robot_vel = 2000;
        ang = 0;
        rot_cclockwise = false;
        w = 0;
        kick_force = 0;
        do_kick = false;
        chip = false;
        forced = false;
        dribble_cclockwise = false;
        dribble_vel = 0;

        ang = 128;

        auto forwardMsg = *createRobotPacket(
                id,
                robot_vel,
                ang,
                rot_cclockwise,
                w,
                kick_force,
                do_kick,
                chip,
                forced,
                dribble_cclockwise,
                dribble_vel
                );

        ang = 256 + 128;

        auto backwardMsg = *createRobotPacket(
                id,
                robot_vel,
                ang,
                rot_cclockwise,
                w,
                kick_force,
                do_kick,
                chip,
                forced,
                dribble_cclockwise,
                dribble_vel
                );

        int batchSize = std::stoi(get_safe_input("Batch size (100):", "100"));
        int const numBytes = 3;
        uint8_t ackCode[numBytes];

        // Forward
        std::cout << "Forward..." << std::flush;
        auto msg = forwardMsg;
        for (int i = 0; i < batchSize; ++i) {
            serialPort.write_some(boost::asio::buffer(msg.data(), msg.size()));

            // TODO: @Hack base station crutches! Pakcet length should be smaller
            serialPort.write_some(boost::asio::buffer(msg.data(), 1));

            // boost::system::error_code err;
            // flush_serial_port(serialPort, flush_type::flush_send, err);

            // if (err != boost::system::errc::success) {
                // std::cout << "Error flushing! Code: " << err.value() << ", message: " << err.message() << "\n";
            // }

            serialPort.read_some(boost::asio::buffer(ackCode, numBytes - 1));

            ackCode[numBytes - 1] = 0;

            std::string returnMessage((char*) &ackCode[0]);

            if (returnMessage[1] == '0') {
                // failCount++;
                std::cout << "X" << std::flush;
            }

            if (i % 100 == 0) {
                std::cout << "." << std::flush;
            }
        }

        // Backward
        std::cout << "Backward..." << std::flush;
        msg = backwardMsg;
        for (int i = 1; i < batchSize + 1; ++i) {
            serialPort.write_some(boost::asio::buffer(msg.data(), msg.size()));

            // TODO: @Hack base station crutches! Pakcet length should be smaller
            serialPort.write_some(boost::asio::buffer(msg.data(), 1));

            // boost::system::error_code err;
            // flush_serial_port(serialPort, flush_type::flush_send, err);

            // if (err != boost::system::errc::success) {
                // std::cout << "Error flushing! Code: " << err.value() << ", message: " << err.message() << "\n";
            // }

            serialPort.read_some(boost::asio::buffer(ackCode, numBytes - 1));

            ackCode[numBytes - 1] = 0;

            std::string returnMessage((char*) &ackCode[0]);

            if (returnMessage[1] == '0') {
                // failCount++;
                std::cout << "X" << std::flush;
            }

            if (i % 100 == 0) {
                std::cout << "." << std::flush;
            }
        }

        std::cout << "Quicktest done!\n";
    } else {
        ////////////////////////
        // Single packet test //
        ////////////////////////

        // if (!get_safe_input("Press enter to continue type something to cancel...").empty()) {
        //     std::cout << "Sending message canceled. Aborting.\n";
        //     return 0;
        // }

        bool keepGoing = true;

        // do {
            // std::cout << "Writing bytes to files... ";

            // serialPort.write_some(boost::asio::buffer(msg.data(), msg.size()));
            boost::asio::write(serialPort, boost::asio::buffer(msg.data(), msg.size()));
            // TODO: @Hack base station crutches! Pakcet length should be smaller
            // serialPort.write_some(boost::asio::buffer(msg.data(), 1));
            boost::asio::write(serialPort, boost::asio::buffer(msg.data(), 1));
            // TODO: @Hack base station crutches! Pakcet length should be smaller

            // std::cout << "Done.\n";


            // if (get_safe_input("Check for ACK (Y/n): ", "Y") != "n") {
                int const numBytes = 3;
                uint8_t ackCode[numBytes];
                // int receivedBytes = serialPort.read_some(boost::asio::buffer(ackCode, numBytes - 1));
                int receivedBytes = boost::asio::read(serialPort, boost::asio::buffer(ackCode, numBytes - 1));

                ackCode[numBytes - 1] = 0;

                // std::cout << "Received bytes: " << receivedBytes << "\n";

                // std::cout << "The byte: " << std::to_string(ackCode[0]) << "\n";
                // std::cout << "The byte: " << std::to_string(ackCode[1]) << "\n";

                std::string returnMessage((char*) &ackCode[0]);
                std::cout << "ACK: " << returnMessage << "\n";
            // }

            // keepGoing = get_safe_input("Send again (Y/n): ", "Y") != "n";
        // } while (keepGoing);
    }

    


	return 0;
}

} // rtt

////////////////////
// Bitch-ass main //
////////////////////

int main(int argc, char *argv[]) {
    std::vector<std::string> arguments(argv + 1, argv + argc);

    while (true) {
    	rtt::main(arguments);
    }

    return 0;
    // return rtt::main(arguments);
}

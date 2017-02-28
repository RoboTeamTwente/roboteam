#include <boost/asio.hpp>
#include <boost/format.hpp>
namespace b = boost;
namespace ba = boost::asio;
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <map>

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

bool portOpened = false;
boost::asio::io_service io;
boost::asio::serial_port serialPort(io);

void printCurrentPacket(LowLevelRobotCommand llrc) {
    std::cout << "\t0\tid: " << (int) llrc.id << "\n";
    std::cout << "\t1\trobot_vel: " << (int) llrc.robot_vel << "\n";
    std::cout << "\t2\tang: " << (int) llrc.ang << "\n";
    std::cout << "\t3\trot_cclockwise: " << (int) llrc.rot_cclockwise << "\n";
    std::cout << "\t4\tw: " << (int) llrc.w << "\n";
    // Indicates power for both kicking & chipping
    std::cout << "\t5\tpunt_power: " << (int) llrc.punt_power << "\n";
    std::cout << "\t6\tdo_kick: " << (int) llrc.do_kick << "\n";
    std::cout << "\t7\tdo_chip: " << (int) llrc.do_chip << "\n";
    std::cout << "\t8\tforced: " << (int) llrc.forced << "\n";
    std::cout << "\t9\tdribble_cclockwise: " << (int) llrc.dribble_cclockwise << "\n";
    std::cout << "\tA\tdribble_vel: " << (int) llrc.dribble_vel << "\n";
}

bool beginsWithOrEquals(std::string input, std::string comparator) {
    return input == comparator || input[0] == comparator[0];
}

bool isInteger(const std::string & s) {
   if(s.empty() || ((!isdigit(s[0])) && (s[0] != '-') && (s[0] != '+'))) return false ;

   char * p ;
   strtol(s.c_str(), &p, 10) ;

   return (*p == 0) ;
}

struct PacketResult {
    std::string rawResult;
    bool success;
    int id;
    bool ack;
} ;

PacketResult trySendPacket(LowLevelRobotCommand llrc, std::string outputFile) {
    if (!portOpened) {
        boost::system::error_code errorCode;
        std::cout << "Opening serial port \n";
        serialPort.open(outputFile, errorCode);

        switch (errorCode.value()) {
            case boost::system::errc::success:
                portOpened = true;
                break;
            default:
                std::cout << "An error occurred while creating the file object. "
                          << "Have you passed the right path? "
                          << "Boost error: " << errorCode << ", message: " << errorCode.message() << "\n";
                break;
        }

        if (!portOpened) {
            PacketResult pr = {};
            return pr;
        }
    }
    
    if (auto robotPacket = createRobotPacket(llrc)) {
        auto msg = *robotPacket;
        ba::write(serialPort, boost::asio::buffer(msg.data(), msg.size()));
        ba::write(serialPort, boost::asio::buffer(msg.data(), 1));

        int const numBytes = 3;
        uint8_t ackCode[numBytes];

        ba::read(serialPort, boost::asio::buffer(ackCode, numBytes - 1));
        ackCode[numBytes - 1] = 0;
        std::string returnMessage((char*) &ackCode[0]);

        PacketResult r;
        r.rawResult = returnMessage;

        if ((returnMessage[1] >= '0' || returnMessage[1] <= '1')
            && ((returnMessage[0] >= '0' && returnMessage[0] <= '9')
               || (returnMessage[0] >= 'A' && returnMessage[0] <= 'F')
               || (returnMessage[0] >= 'a' && returnMessage[0] <= 'f')
               )
           ) {
            r.ack = returnMessage[1] == '1';

            if (returnMessage[0] >= '0' && returnMessage[0] <= '9') {
                r.id = returnMessage[0] - '0';
            } else if (returnMessage[0] >= 'A' && returnMessage[0] <= 'F') {
                r.id = returnMessage[0] - 'A';
            } else if (returnMessage[0] >= 'a' && returnMessage[0] <= 'f') {
                r.id = returnMessage[0] - 'a';
            }
        } else {
            r.success = false;
        }

        return r;
    } else {
        std::cout << "Error: could not convert LowLevelRobotCommand into a robotPacket\n";

        PacketResult r;
        r.success = false;
        return r;
    }
}

std::map<std::string, LowLevelRobotCommand> sampleCommands = {
    {"f", {    // forward
        7,     // id 
        2000,  // robot_vel 
        128,     // ang 
        false, // rot_cclockwise 
        0,     // w 
        0,     // punt_power
        false, // do_kick 
        false, // do_chip 
        false, // forced 
        false, // dribble_cclockwise 
        0      // dribble_vel 
    }},
    {"b", {    // backward
        7,     // id 
        2000,  // robot_vel 
        128 + 256,     // ang 
        false, // rot_cclockwise 
        0,     // w 
        0,     // punt_power
        false, // do_kick 
        false, // do_chip 
        false, // forced 
        false, // dribble_cclockwise 
        0      // dribble_vel 
    }},
    {"l", {    // left
        7,     // id 
        2000,  // robot_vel 
        256,     // ang 
        false, // rot_cclockwise 
        0,     // w 
        0,     // punt_power
        false, // do_kick 
        false, // do_chip 
        false, // forced 
        false, // dribble_cclockwise 
        0      // dribble_vel 
    }},
    {"r", {    // right
        7,     // id 
        2000,  // robot_vel 
        0,     // ang 
        false, // rot_cclockwise 
        0,     // w 
        0,     // punt_power
        false, // do_kick 
        false, // do_chip 
        false, // forced 
        false, // dribble_cclockwise 
        0      // dribble_vel 
    }},
    {"s", {    // stop
        7,     // id 
        0,     // robot_vel 
        0,     // ang 
        false, // rot_cclockwise 
        0,     // w 
        0,     // punt_power
        false, // do_kick 
        false, // do_chip 
        false, // forced 
        false, // dribble_cclockwise 
        0      // dribble_vel 
    }},
    {"k", {    // kick
        7,     // id 
        0,     // robot_vel 
        0,     // ang 
        false, // rot_cclockwise 
        0,     // w 
        200,   // punt_power
        true,  // do_kick 
        false, // do_chip 
        true,  // forced 
        false, // dribble_cclockwise 
        0      // dribble_vel 
    }},
    {"d7", {   // dribble on 7
        7,     // id 
        0,     // robot_vel 
        0,     // ang 
        false, // rot_cclockwise 
        0,     // w 
        0,     // punt_power
        false, // do_kick 
        false, // do_chip 
        false, // forced 
        false, // dribble_cclockwise 
        7      // dribble_vel 
    }},
    {"d4", {   // dribble on 4
        7,     // id 
        0,     // robot_vel 
        0,     // ang 
        false, // rot_cclockwise 
        0,     // w 
        0,     // punt_power
        false, // do_kick 
        false, // do_chip 
        false, // forced 
        false, // dribble_cclockwise 
        7      // dribble_vel 
    }},
    {"d1", {   // dribble on 1
        7,     // id 
        0,     // robot_vel 
        0,     // ang 
        false, // rot_cclockwise 
        0,     // w 
        0,     // punt_power
        false, // do_kick 
        false, // do_chip 
        false, // forced 
        false, // dribble_cclockwise 
        7      // dribble_vel 
    }},
    {"d0", {   // dribble off
        7,     // id 
        0,     // robot_vel 
        0,     // ang 
        false, // rot_cclockwise 
        0,     // w 
        0,     // punt_power
        false, // do_kick 
        false, // do_chip 
        false, // forced 
        false, // dribble_cclockwise 
        0      // dribble_vel 
    }},
    {"c", {    // chip
        7,     // id 
        0,     // robot_vel 
        0,     // ang 
        false, // rot_cclockwise 
        0,     // w 
        200,   // punt_power
        false, // do_kick 
        true,  // do_chip 
        true,  // forced 
        false, // dribble_cclockwise 
        0      // dribble_vel 
    }}
};

std::string sampleCommandsText = R"--(Sample commands:
    forward (f)
    backward (b)
    left (l)
    right (r)
    stop (s)
    kick (k)
    dribble on (d1/d4/d7)
    dribble off (d0)
    chip (c))--";

b::optional<LowLevelRobotCommand> trySampleCommand(std::string instruction) {
    if (instruction.size() == 0) {
        return b::none;
    }

    if (instruction == "dribble on") {
        instruction = "d4";
    } else if (instruction == "dribble off") {
        instruction = "d0";
    } else if (instruction[0] != 'd') {
        instruction = instruction.substr(0, 1);
    }

    auto it = sampleCommands.find(instruction);
    if (it == sampleCommands.end()) {
        return b::none;
    }

    return it->second;
}

void sendCommand(LowLevelRobotCommand llrc, std::string outputFile, std::vector<LowLevelRobotCommand>& history) {
    auto pr = trySendPacket(llrc, outputFile);

    if (pr.success) {
        std::cout << "Packet sent successfully to robot #"
            << pr.id
            << ", response: "
            << (pr.ack ? "ACK" : "NACK")
            << "\n";
    } else {
        std::cout << "Packet error\n";
    }
    
    if (history.size() > 0) {
        if (history.back() != llrc) {
            history.push_back(llrc);
        }
    } else {
        history.push_back(llrc);
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

    if (std::find(arguments.begin(), arguments.end(), "--help") != arguments.end()) {
        std::cout << "Arguments: either a port (/dev/ttyACM0 or something), --help for this, or msg to test the roboteam_msg to robot packet mechanism.\n";
        exit(0);
    }

    std::string outputFile = arguments.at(0);

    int robotID = std::stoi(arguments.at(1));

    std::cout << "Output file: " << outputFile << " robotID: " << robotID << "\n";

    bool quit = false;
    std::vector<LowLevelRobotCommand> history;

    LowLevelRobotCommand llrc = {};
    llrc.id = robotID;

    do {
        std::cout << "Current packet:\n";

        printCurrentPacket(llrc);

        std::cout << sampleCommandsText << "\n";

        std::string instruction = get_safe_input("rtt> ", "send");
        std::transform(instruction.begin(), instruction.end(), instruction.begin(), ::tolower);

        if (beginsWithOrEquals(instruction, "quit")) {
            quit = true;
        } else if (instruction == "send") {
            sendCommand(llrc, outputFile, history);
        } else if (auto possibleLLRC = trySampleCommand(instruction)) {

            auto sampleLLRC = *possibleLLRC;
            sampleLLRC.id = robotID;

            sendCommand(sampleLLRC, outputFile, history);
        } else {
            std::cout << "Unknown command\n";
        }
    } while (!quit);

    /////////////////////
    // Creating packet //
    /////////////////////


    


    



    // auto packetType = get_safe_input(R"--(Packet type options:

    // // TODO: w should be ang, w_vel should be w
    // int id;
    // int robot_vel;
    // int ang;
    // bool rot_cclockwise;
    // int w;
    // uint8_t kick_force;
    // bool do_kick;
    // bool chip;
    // bool forced;
    // bool dribble_cclockwise;
    // uint8_t dribble_vel;

    // if (packetType == "manual") {
        // std::cout << "Sending a manual packet.\n";

        // id                 = std::stoi(get_safe_input("id (0-15, 7): ", "7"));
        // robot_vel          = std::stoi(get_safe_input("robot_vel (0-8191, 2000): ", "2000"));
        // ang                = std::stoi(get_safe_input("ang (0-511, 300): ", "300"));
        // rot_cclockwise     = get_safe_input("rot_cclockwise (true/false): ") == "true";
        // w                  = std::stoi(get_safe_input("w (0-2047, 1000): ", "1000"));
        // kick_force         = std::stoi(get_safe_input("kick_force (0-255, 200): ", "200"));
        // do_kick            = get_safe_input("do_kick (true/false): ") == "true";
        // chip               = get_safe_input("chip (true/false): ") == "true";
        // forced             = get_safe_input("forced (true/false): ") == "true";
        // dribble_cclockwise = get_safe_input("dribble_cclockwise (true/false): ") == "true";
        // dribble_vel        = std::stoi(get_safe_input("dribble_vel (0-7, 5): ", "5"));
    // } else if (packetType == "EXAMPLE") {
        // std::cout << "Sending an example packet.\n";
        
        // id = 7;
        // robot_vel = 2000;
        // ang = 300;
        // rot_cclockwise = true;
        // w = 1000;
        // kick_force = 200;
        // do_kick = true;
        // chip = false;
        // forced = true;
        // dribble_cclockwise = true;
        // dribble_vel = 5;
    // } else if (packetType == "f" || packetType == "b" || packetType == "l" || packetType == "r" || 
               // packetType == "fl" || packetType == "fr" || packetType == "bl" || packetType == "br" ) {
        // id = robotID;
        // robot_vel = 2000;
        // ang = 0;
        // rot_cclockwise = false;
        // w = 0;
        // if (packetType == "f") {
            // rot_cclockwise = false;
            // w = 10;
        // } else if (packetType == "b") {
            // rot_cclockwise = true;
            // w = 10;
        // }
        
        
        // kick_force = 0;
        // do_kick = false;
        // chip = false;
        // forced = false;
        // dribble_cclockwise = false;
        // dribble_vel = 0;

        // if (packetType == "f") {
            // ang = 128;
        // } else if (packetType == "b") {
            // ang = 384;
        // } else if (packetType == "l") {
            // ang = 256;
        // } else if (packetType == "r") {
            // ang = 0;
        // } else if (packetType == "fl") {
            // ang = 192;
        // } else if (packetType == "fr") {
            // ang = 64;
        // } else if (packetType == "bl") {
            // ang = 320;
        // } else if (packetType == "br") {
            // ang = 448;
        // }

    // } else if (packetType == "s") {
        // id = robotID;
        // robot_vel = 0;
        // ang = 0;
        // rot_cclockwise = false;
        // w = 0;
        // kick_force = 0;
        // do_kick = false;
        // chip = false;
        // forced = false;
        // dribble_cclockwise = false;
        // dribble_vel = 0;
    // } else if (packetType == "k1" || packetType == "kick1") {
        // id = robotID;
        // robot_vel = 0;
        // ang = 0;
        // rot_cclockwise = true;
        // w = 0;
        // kick_force = 128;
        // do_kick = true;
        // chip = false;
        // forced = true;
        // dribble_cclockwise = 0;
        // dribble_vel = 0;
    // } else if (packetType == "k2" || packetType == "kick2") {
        // id = robotID;
        // robot_vel = 0;
        // ang = 0;
        // rot_cclockwise = true;
        // w = 0;
        // kick_force = 255;
        // do_kick = true;
        // chip = false;
        // forced = true;
        // dribble_cclockwise = 0;
        // dribble_vel = 0;
    // } else if (packetType == "d1" || packetType == "d4" || packetType == "d7" || packetType == "dribble on") {
        // id = robotID;
        // robot_vel = 0;
        // ang = 0;
        // rot_cclockwise = false;
        // w = 0;
        // kick_force = 0;
        // do_kick = false;
        // chip = false;
        // forced = false;
        // dribble_cclockwise = 0;
        // dribble_vel = 1;

        // if (packetType == "d1") {
            // dribble_vel = 1;
        // } else if (packetType == "d4") {
            // dribble_vel = 4;
        // } else if (packetType == "d7") {
            // dribble_vel = 7;
        // }
    // } else if (packetType == "d0" || packetType == "dribble off") {
        // id = robotID;
        // robot_vel = 0;
        // ang = 0;
        // rot_cclockwise = true;
        // w = 0;
        // kick_force = 0;
        // do_kick = false;
        // chip = false;
        // forced = false;
        // dribble_cclockwise = 0;
        // dribble_vel = 0;
    // } else if (packetType == "c" || packetType == "chip") {
        // id = robotID;
        // robot_vel = 0;
        // ang = 0;
        // rot_cclockwise = true;
        // w = 0;
        // kick_force = 255;
        // do_kick = false;
        // chip = true;
        // forced = true;
        // dribble_cclockwise = 0;
        // dribble_vel = 0;
    // }


    // /////////////////////
    // // Printing packet //
    // /////////////////////

    // // #define FIELD(id) std::cout << bf::format("\t%-20s %-20s\n") % #id % get_pretty_value(id);

    // // std::cout << "Packet:\n";
    // // FIELD(id);
    // // FIELD(robot_vel);
    // // FIELD(ang);
    // // FIELD(rot_cclockwise);
    // // FIELD(w);
    // // FIELD(kick_force);
    // // FIELD(do_kick);
    // // FIELD(chip);
    // // FIELD(forced);
    // // FIELD(dribble_cclockwise);
    // // FIELD(dribble_vel);

    // std::cout << "Creating message... ";

    // ///////////////////////
    // // Converting packet //
    // ///////////////////////

    // auto possibleMsg = createRobotPacket(
            // id,
            // robot_vel,
            // ang,
            // rot_cclockwise,
            // w,
            // kick_force,
            // do_kick,
            // chip,
            // forced,
            // dribble_cclockwise,
            // dribble_vel
            // );

    // if (!possibleMsg) {
        // std::cout << "An error occurred while creating the message. Please look at the constraints of createRobotPacket(). Aborting.";
        // return 1;
    // }

    // // std::cout << "All params ok.\n";

    // ////////////////////////////
    // // Printing binary packet //
    // ////////////////////////////

    // // std::cout << "Message contents: \n";

    // auto msg = *possibleMsg;

    // // for (const auto& byte : msg) {
    // //     std::cout << "\t" << byteToBinary(byte) << "\t" << std::to_string(byte) << "\n";
    // // }

    // //////////////////////////
    // // Creating serial port //
    // //////////////////////////

    // // if (!get_safe_input("Press enter to open the port or type something to cancel...").empty()) {
    // //     std::cout << "Sending message canceled. Aborting.\n";
    // //     return 0;
    // // }

    // // std::cout << "Creating serial port... ";


    // // std::cout << "Done.\n";

    // ///////////////////////
    // // Setting baud rate //
    // ///////////////////////

    // boost::asio::serial_port_base::baud_rate baudRate;
    // serialPort.get_option(baudRate);
    // // std::cout << "Baud rate: " << baudRate.value() << "\n";
    
    // // std::cout << "Other standard baud rates: \n";
    // // for (auto br : baudRates) {
    // //     std::cout << "    " << br << "\n";
    // // }
    // // std::cout << "Less standard baud rates: \n";
    // // for (auto br : nonStandardBaudRates) {
    // //     std::cout << "    " << br << "\n";
    // // }

    // // auto userBaudRateStr = get_safe_input("Set baud rate (or empty to leave at standard): ");
    // // if (!userBaudRateStr.empty()) {
    // //     try {
    // //         int userBaudRate = std::stoi(userBaudRateStr);
    // //         baudRate = boost::asio::serial_port_base::baud_rate(userBaudRate);
    // //         boost::system::error_code ec;
    // //         serialPort.set_option(baudRate, ec);

    // //         if (ec != boost::system::errc::success) {
    // //             std::cout << "Could not set baud rate. Error: " << ec << ". " << ec.message() << "\n";
    // //         }

    // //         serialPort.get_option(baudRate);
    // //         std::cout << "Final baud rate: " << baudRate.value() << "\n";
    // //     } catch (std::invalid_argument const & ex) {
    // //         std::cout << "Invalid baudRate specified. Continuing with baudrate " << baudRate.value() << "\n";
    // //     } catch (std::out_of_range const & ex) {
    // //         std::cout << "Specified baud rate too large for an int. Continuing with baudrate " << baudRate.value() << "\n";
    // //     }
    // // }

    // bool benchMark = get_safe_input("Benchmark (y/N)? ", "N") != "N";
    // // bool benchMark = false;

    
    // bool quickTest = get_safe_input("Quick test? (y/N) ", "N") != "N";

    // if (benchMark) {
        // ///////////////
        // // Benchmark //
        // ///////////////

        // bool checkForAck = get_safe_input("Check for ACK (Y/n): ", "Y") != "n";
        // int const MESSAGE_QUANTITY = std::stoi(get_safe_input("Amount of messages (10 000): ", "10000"));

        // int const numBytes = 3;
        // uint8_t ackCode[numBytes];

        // if (!get_safe_input("Press enter to continue type something to cancel...").empty()) {
            // std::cout << "Sending message canceled. Aborting.\n";
            // return 0;
        // }
        
        // std::cout << "Sending packets..." << std::flush;

        // // Communicatie todo
        // // TODO: Make packet length 7 in hans's stuff
        // // TODO: Make library of hans nrf stuff
        // // gedaan: serial baud testing
        // // gedaan: SPI baud testing
        // // TODO: Make main pcb response 2 bytes instead of 2 ascii characters
        // // gedaan: Optization for hans' code, useful or not? Nee
        // // gedaan: WriteData optimaliseren bij Hans? Semi-lelijk

        // // There are other clocks, but this is usually the one you want.
        // // It corresponds to CLOCK_MONOTONIC at the syscall level.
        // using Clock = std::chrono::steady_clock;
        // using std::chrono::time_point;
        // using std::chrono::duration_cast;
        // using std::chrono::milliseconds;
        // using namespace std::literals::chrono_literals;
        // using std::this_thread::sleep_for;

        // time_point<Clock> start = Clock::now();

        // int failCount = 0;

        // for (int i = 1; i < MESSAGE_QUANTITY + 1; ++i) {
            // serialPort.write_some(boost::asio::buffer(msg.data(), msg.size()));

            // // TODO: @Hack base station crutches! Pakcet length should be smaller
            // serialPort.write_some(boost::asio::buffer(msg.data(), 1));

            // // boost::system::error_code err;
            // // flush_serial_port(serialPort, flush_type::flush_send, err);

            // // if (err != boost::system::errc::success) {
                // // std::cout << "Error flushing! Code: " << err.value() << ", message: " << err.message() << "\n";
            // // }

            // if (checkForAck) {
                // serialPort.read_some(boost::asio::buffer(ackCode, numBytes - 1));

                // ackCode[numBytes - 1] = 0;

                // std::string returnMessage((char*) &ackCode[0]);

                // if (returnMessage[1] == '0') {
                    // failCount++;
                // }
            // }

            // if (i % 100 == 0) {
                // std::cout << "." << std::flush;
            // }
        // }

        // time_point<Clock> end = Clock::now();

        // milliseconds diff = duration_cast<milliseconds>(end - start);

        // std::cout << "\nBenchmark done!\n";
        // std::cout << "Sent " << MESSAGE_QUANTITY << " messages.\n";
        // std::cout << "Duration: " << diff.count() << "ms" << std::endl;
        // std::cout << "Failures: " << failCount << "\n";
    // } else if (quickTest) {
        // std::cout << "Quicktest!\n";

        // id = robotID;
        // robot_vel = 0;
        // ang = 0;
        // rot_cclockwise = false;
        // w = 70;
        // kick_force = 0;
        // do_kick = false;
        // chip = false;
        // forced = false;
        // dribble_cclockwise = false;
        // dribble_vel = 0;

        // ang = 0;

        // auto forwardMsg = *createRobotPacket(
                // id,
                // robot_vel,
                // ang,
                // rot_cclockwise,
                // w,
                // kick_force,
                // do_kick,
                // chip,
                // forced,
                // dribble_cclockwise,
                // dribble_vel
                // );

        // ang = 0;

        // w = 40;

        // auto backwardMsg = *createRobotPacket(
                // id,
                // robot_vel,
                // ang,
                // rot_cclockwise,
                // w,
                // kick_force,
                // do_kick,
                // chip,
                // forced,
                // dribble_cclockwise,
                // dribble_vel
                // );

        // // int batchSize = std::stoi(get_safe_input("Batch size (100):", "100"));
        // int const numBytes = 3;
        // uint8_t ackCode[numBytes];

        // // Forward
        // std::cout << "Fast..." << std::flush;
        // auto msg = forwardMsg;
        // for (int i = 0; i < 0; ++i) {
            // serialPort.write_some(boost::asio::buffer(msg.data(), msg.size()));

            // // TODO: @Hack base station crutches! Pakcet length should be smaller
            // serialPort.write_some(boost::asio::buffer(msg.data(), 1));

            // // boost::system::error_code err;
            // // flush_serial_port(serialPort, flush_type::flush_send, err);

            // // if (err != boost::system::errc::success) {
                // // std::cout << "Error flushing! Code: " << err.value() << ", message: " << err.message() << "\n";
            // // }

            // serialPort.read_some(boost::asio::buffer(ackCode, numBytes - 1));

            // ackCode[numBytes - 1] = 0;

            // std::string returnMessage((char*) &ackCode[0]);

            // if (returnMessage[1] == '0') {
                // // failCount++;
                // // std::cout << "X" << std::flush;
            // } else {
                // std::cout << "." << std::flush;
            // }

            // // if (i % 100 == 0) {
                // // std::cout << "." << std::flush;
            // // }
        // }

        // // Backward
        // std::cout << "Slow..." << std::flush;
        // msg = backwardMsg;
        // for (int i = 1; i < 1000 + 1; ++i) {
            // serialPort.write_some(boost::asio::buffer(msg.data(), msg.size()));

            // // TODO: @Hack base station crutches! Pakcet length should be smaller
            // serialPort.write_some(boost::asio::buffer(msg.data(), 1));

            // // boost::system::error_code err;
            // // flush_serial_port(serialPort, flush_type::flush_send, err);

            // // if (err != boost::system::errc::success) {
                // // std::cout << "Error flushing! Code: " << err.value() << ", message: " << err.message() << "\n";
            // // }

            // serialPort.read_some(boost::asio::buffer(ackCode, numBytes - 1));

            // ackCode[numBytes - 1] = 0;

            // std::string returnMessage((char*) &ackCode[0]);

            // if (returnMessage[1] == '0') {
                // // failCount++;
                // // std::cout << "X" << std::flush;
            // } else {

            // // if (i % 100 == 0) {
                // std::cout << "." << std::flush;
            // // }
            // }
        // }

        // std::cout << "Quicktest done!\n";
    // } else {
        // ////////////////////////
        // // Single packet test //
        // ////////////////////////

        // // if (!get_safe_input("Press enter to continue type something to cancel...").empty()) {
        // //     std::cout << "Sending message canceled. Aborting.\n";
        // //     return 0;
        // // }

        // bool keepGoing = true;

        // // do {
            // // std::cout << "Writing bytes to files... ";

            // // serialPort.write_some(boost::asio::buffer(msg.data(), msg.size()));
            // boost::asio::write(serialPort, boost::asio::buffer(msg.data(), msg.size()));
            // // TODO: @Hack base station crutches! Pakcet length should be smaller
            // // serialPort.write_some(boost::asio::buffer(msg.data(), 1));
            // boost::asio::write(serialPort, boost::asio::buffer(msg.data(), 1));
            // // TODO: @Hack base station crutches! Pakcet length should be smaller

            // // std::cout << "Done.\n";


            // // if (get_safe_input("Check for ACK (Y/n): ", "Y") != "n") {
                // int const numBytes = 3;
                // uint8_t ackCode[numBytes];
                // // int receivedBytes = serialPort.read_some(boost::asio::buffer(ackCode, numBytes - 1));
                // int receivedBytes = boost::asio::read(serialPort, boost::asio::buffer(ackCode, numBytes - 1));

                // ackCode[numBytes - 1] = 0;

                // // std::cout << "Received bytes: " << receivedBytes << "\n";

                // // std::cout << "The byte: " << std::to_string(ackCode[0]) << "\n";
                // // std::cout << "The byte: " << std::to_string(ackCode[1]) << "\n";

                // std::string returnMessage((char*) &ackCode[0]);
                // std::cout << "ACK: " << returnMessage << "\n";
            // // }

            // // keepGoing = get_safe_input("Send again (Y/n): ", "Y") != "n";
        // // } while (keepGoing);
    // }

    


	return 0;
}

} // rtt

////////////////////
// Bitch-ass main //
////////////////////

int main(int argc, char *argv[]) {
    std::vector<std::string> arguments(argv + 1, argv + argc);

    rtt::main(arguments);

    return 0;
    // return rtt::main(arguments);
}


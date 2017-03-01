#include <boost/asio.hpp>
#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>
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

namespace b = boost;
namespace ba = boost::asio;

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

        std::cout << "Sent packet: \n";
        printCurrentPacket(llrc);

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

int getSafeInt(std::string question, int defaultValue) {
    std::string rawInput = get_safe_input(question, std::to_string(defaultValue));

    try {
        return std::stoi(rawInput);
    } catch (...) {
        std::cout << "Could not parse input \"" << rawInput << "\". Defaulting to " << defaultValue << "\n";
        return defaultValue;
    }
}

LowLevelRobotCommand getLowLevelRobotCommandFromInput() {
    std::cout << "Please enter all the desired values\n";

    LowLevelRobotCommand llrc = {};
    
    llrc.id                 = getSafeInt("\tid (0-15, 0): ", 0);
    llrc.robot_vel          = getSafeInt("\trobot_vel (0-8191, 0): ", 0);
    llrc.ang                = getSafeInt("\tang (0-511, 0): ", 0);
    llrc.rot_cclockwise     = getSafeInt("\trot_cclockwise (0 or > 0, 0): ", 0) > 0;
    llrc.w                  = getSafeInt("\tw (0-2047, 0): ", 0);
    llrc.punt_power         = getSafeInt("\tpunt_power (0-255, 0): ", 0);
    llrc.do_kick            = getSafeInt("\tdo_kick (0 or > 0, 0): ", 0) > 0;
    llrc.do_chip            = getSafeInt("\tdo_chip (0 or > 0, 0): ", 0) > 0;
    llrc.forced             = getSafeInt("\tforced (0 or > 0, 0): ", 0) > 0;
    llrc.dribble_cclockwise = getSafeInt("\tdribble_cclockwise (0 or > 0, 0): ", 0) > 0;
    llrc.dribble_vel        = getSafeInt("\tdribble_vel (0-7, 0): ", 0);

    return llrc;
}

std::vector<std::string> llrcKeys = {
    "id",
    "robot_vel",
    "ang",
    "rot_cclockwise",
    "w",
    "punt_power",
    "do_kick",
    "do_chip",
    "forced",
    "dribble_cclockwise",
    "dribble_vel"
};

b::optional<std::tuple<std::string, int>> parseAssignment(std::string instruction) {
    auto eqPos = instruction.find("=");
    if (eqPos == std::string::npos) {
        return b::none;
    } 

    std::string index = b::trim_copy(instruction.substr(0, eqPos));
    std::string value = b::trim_copy(instruction.substr(eqPos + 1));

    auto keyIt = std::find(llrcKeys.begin(), llrcKeys.end(), index);
    if (keyIt != llrcKeys.end()) {
        index = std::to_string(keyIt - llrcKeys.begin());
    }

    try {
        if (isInteger(index) || index == "A" || index == "a") {
            if (index == "A" || index == "a") {
                return std::make_tuple(llrcKeys.at(9), std::stoi(value));
            }

            int indexInt = std::stoi(index);

            if (indexInt < 0 || indexInt > 9) {
                std::cout << "Index " << indexInt << "is an invalid index\n";
                return b::none;
            }

            return std::make_tuple(llrcKeys.at(indexInt), std::stoi(value));
        } else {
            return std::make_tuple(index, std::stoi(value));
        }
    } catch (...) {
        return b::none;
    }
}

LowLevelRobotCommand applyAssignment(std::tuple<std::string, int> assignmentInfo, LowLevelRobotCommand llrc) {
    auto index = std::get<0>(assignmentInfo);
    auto value = std::get<1>(assignmentInfo);

    if (index == "id") { 
        if (value < 0 || value > 15) {
            std::cout << "id " << value << " has to be between 0 and 15 inclusive\n";
            return llrc;
        }

        llrc.id = value;
    } else if (index == "robot_vel") { 
        if (value < 0 || value > 8191) {
            std::cout << "robot_vel has to be between 0 and 8191 inclusive\n";
            return llrc;
        }

        llrc.robot_vel = value;
    } else if (index == "ang") {
        if (value < 0 || value > 511) {
            std::cout << "ang has to be between 0 and 511 inclusive\n";
            return llrc;
        }

        llrc.ang = value;
    } else if (index == "rot_cclockwise") { 
        llrc.rot_cclockwise = value > 0;
    } else if (index == "w") { 
        if (value < 0 || value > 2047) {
            std::cout << "w has to be between 0 and 2047 inclusive\n";
            return llrc;
        }

        llrc.w = value;
    } else if (index == "punt_power") {
        if (value < 0 || value > 255) {
            std::cout << "punt_power has to be between 0 and 255 inclusive\n";
            return llrc;
        }

        llrc.punt_power = value;
    } else if (index == "do_kick") {
        llrc.do_kick = value > 0;
    } else if (index == "do_chip") {
        llrc.do_chip = value > 0;
    } else if (index == "forced") {
        llrc.forced = value > 0;
    } else if (index == "dribble_cclockwise") { 
        llrc.dribble_cclockwise = value > 0;
    } else if (index == "dribble_vel") {
        if (value < 0 || value > 7) {
            std::cout << "dribble_vel has to be between 0 and 7 inclusive\n";
            return llrc;
        }

        llrc.dribble_vel = value;
    } else {
        std::cout << "LowLevelRobotCommand variablename " << index << " is invalid\n";
        return llrc;
    }

    std::cout << value << " => " << index << "\n";

    return llrc;
}
    
} // anonymous namespace

int main(const std::vector<std::string>& arguments) {
    // TODO: Print help! And add (h)elp command

    if (arguments.size() < 2) {
        std::cout << "Too few arguments, please specify output file and robotID. Aborting.\n";
        return 1;
    }

    if (std::find(arguments.begin(), arguments.end(), "--help") != arguments.end()) {
        std::cout << "Arguments: either a port (/dev/ttyACM0 or something), --help for this, or msg to test the roboteam_msg to robot packet mechanism.\n";
        exit(0);
    }

    std::string outputFile = arguments.at(0);

    LowLevelRobotCommand llrc = {};
    llrc.id = std::stoi(arguments.at(1));

    std::cout << "Output file: " << outputFile << " robotID: " << llrc.id << "\n";

    bool quit = false;
    std::vector<LowLevelRobotCommand> history;

    do {
        std::cout << 1 + R"##(
---------------------------
-- RTT Packet Tester CMD --
---------------------------
)##";
        std::cout << "Current packet:\n";

        printCurrentPacket(llrc);

        std::cout << sampleCommandsText << "\n";

        std::string instruction = get_safe_input("rtt> ", "send");
        std::transform(instruction.begin(), instruction.end(), instruction.begin(), ::tolower);

        if (beginsWithOrEquals(instruction, "quit")) {
            quit = true;
        } else if (instruction == "manual") {
            llrc = getLowLevelRobotCommandFromInput();
        } else if (instruction == "send") {
            sendCommand(llrc, outputFile, history);
        } else if (auto assignmentOpt = parseAssignment(instruction)) {
            auto assignmentInfo = *assignmentOpt;
            
            llrc = applyAssignment(assignmentInfo, llrc);
        } else if (auto possibleLLRC = trySampleCommand(instruction)) {

            auto sampleLLRC = *possibleLLRC;
            sampleLLRC.id = llrc.id;

            sendCommand(sampleLLRC, outputFile, history);
        } else if (instruction[0] == '-') {
            auto historyStr = instruction.substr(1, instruction.size() - 1);

            if (!isInteger(historyStr)) {
                std::cout << "Could not parse historycommand. Please use the format \"-###\", where ### is a number between 1 and infinity\n";
                continue;
            }

            // Between 1 and inf
            int historyNum = std::stoi(historyStr);
            // Between 0 and inf
            int historyIndex = historyNum - 1;

            if (!(historyIndex >= 0 && historyIndex < history.size())) {
                std::cout << "Cannot look back in the history " << historyNum << " steps\n";
                continue;
            } 
            
            sendCommand(history[historyIndex], outputFile, history);
        } else {
            std::cout << "Unknown command\n";
        }
    } while (!quit);

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


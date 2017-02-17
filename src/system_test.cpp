#include <boost/asio.hpp>
#include <boost/format.hpp>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "roboteam_robothub/packing.h"

namespace rtt {

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

boost::asio::io_service io;
boost::asio::serial_port serialPort(io);

packed_protocol_message makeStop(int id) {
    return *createRobotPacket(
        id,
        0,
        0,
        false,
        0,
        0,
        false,
        false,
        false,
        false,
        0
        );
}

packed_protocol_message makeDirectional(int id, int robot_vel, int ang) {
    return *createRobotPacket(
        id,
        robot_vel,
        ang,
        false,
        0,
        0,
        false,
        false,
        false,
        false,
        0
        );
}

packed_protocol_message makeRotational(int id, bool rot_cclockwise, int w) {
    return *createRobotPacket(
        id,
        0,
        0,
        rot_cclockwise,
        w,
        0,
        false,
        false,
        false,
        false,
        0
        );
}

packed_protocol_message makeKick(int id) {
    return *createRobotPacket(
        id,
        0,
        0,
        false,
        0,
        100,  // 100 punt power
        true, // enable kick
        false,
        true, // USE THE FORCE, PROTOBOT
        false,
        0
        );
}

struct Instruction {
    std::string name;
    packed_protocol_message packet;
} ;

}

int main(const std::vector<std::string>& arguments) {
    if (arguments.size() < 1) {
        std::cout << "No output file specified as argument. Aborting.\n";
        return 1;
    }

    std::string output_file = arguments.at(0);

    std::cout << "Output file: " << output_file << "\n";
    if (!get_safe_input("Press enter to open the port or type something to cancel...").empty()) {
        std::cout << "Sending message canceled. Aborting.\n";
        return 0;
    }

    std::string robotIdStr = get_safe_input("Enter robot id (0-15, 0): ", "0");
    int robotID = 0;
    try {
        robotID = std::stoi(robotIdStr);
    } catch(...) {
        std::cout << "Could not parse given robot ID. Using 0.\n";
    }

    std::string robotVelStr = get_safe_input("Enter robot velocity in mm/s (0-8191, 1000): ", "1000");
    int robotVel = 1000;
    try {
        robotVel = std::stoi(robotVelStr);
    } catch (...) {
        std::cout << "Could not parse given robot velocity, using 1000 mm/s.\n";
    }

    std::string wStr = get_safe_input("Enter angular velocity in deg/s (0-2047, 100): ", "100");
    int w = 100;
    try {
        w = std::stoi(wStr);
    } catch (...) {
        std::cout << "Could not parse angular velocity, using 100 deg/s.\n";
    }

    if(!get_safe_input("Press enter to continue or type anything to exit...").empty()) {
        std::cout << "Aborting\n";
        exit(0);
    }

    std::cout << "Creating serial port... ";

    boost::system::error_code errorCode;
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

    std::cout << "Done.\n";

    if(!get_safe_input("Press enter to start system test or type anything to exit...").empty()) {
        std::cout << "Aborting\n";
        exit(0);
    }

    Instruction stop  = {"stop",         makeStop(robotID)                      };
    Instruction up    = {"up",           makeDirectional(robotID, robotVel, 128)};
    Instruction down  = {"down",         makeDirectional(robotID, robotVel, 128)};
    Instruction left  = {"left",         makeDirectional(robotID, robotVel, 128)};
    Instruction right = {"right",        makeDirectional(robotID, robotVel, 128)};
    Instruction a     = {"rotate left",  makeRotational(robotID, true, w)       };
    Instruction b     = {"rotate right", makeRotational(robotID, false, w)      };
    Instruction start = {"kick",         makeKick(robotID)                      };

    std::vector<Instruction> instructions = {
        up,    stop,
        up,    stop,
        down,  stop,
        down,  stop,
        left,  stop,
        right, stop,
        left,  stop,
        right, stop,
        a,     stop,
        b,     stop,
        start, stop
    };

    // TODO: Playback!
}

} // rtt

int main(int argc, char *argv[]) {
    std::vector<std::string> arguments(argv + 1, argv + argc);

    return rtt::main(arguments);
}

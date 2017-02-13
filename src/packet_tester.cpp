#include <cstdint>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <boost/format.hpp>
#include <boost/asio.hpp>

#include "roboteam_robothub/packing.h"

namespace rtt {

std::string get_safe_input(std::string question = "", bool quit_on_empty = true) {
    if (!question.empty()) {
        std::cout << question;
    }

    std::string input_str;
    
    if(std::getline(std::cin, input_str)) {
        if (input_str.empty() && quit_on_empty) {
            std::cout << "\nEmpty string passed as input. Aborting.\n";
            exit(1);
        }

        return input_str;
    }

    std::cout << "\nIO error. Aborting.\n";
    exit(1);
}

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

// http://www.boost.org/doc/libs/1_40_0/doc/html/boost_asio/overview/serial_ports.html

boost::asio::io_service io;
boost::asio::serial_port serialPort(io);

} // anonymous namespace

int main(const std::vector<std::string>& arguments) {
    if (arguments.size() < 1) {
        std::cout << "No output file specified as argument. Aborting.\n";
        return 1;
    }

    std::string output_file = arguments.at(0);

    std::cout << "Output file: " << output_file << "\n";

    bool manual = get_safe_input("Example packet or initialize packet by hand (EXAMPLE/manual)? ") == "manual";

    // TODO: w should be ang, w_vel should be w
    int id;
    int robot_vel;
    int w;
    bool rot_cclockwise;
    int w_vel;
    uint8_t kick_force;
    bool do_kick;
    bool chip;
    bool forced;
    bool dribble_cclockwise;
    uint8_t dribble_vel;

    if (manual) {
        std::cout << "Sending a manual packet.\n";

        id = std::stoi(get_safe_input("id (0-15): "));
        robot_vel = std::stoi(get_safe_input("robot_vel (0-4095): "));
        w = std::stoi(get_safe_input("w (0-511): "));
        rot_cclockwise = get_safe_input("rot_cclockwise (true/false): ") == "true";
        w_vel = std::stoi(get_safe_input("w_vel (0-2047): "));
        kick_force = std::stoi(get_safe_input("kick_force (0-255): "));
        do_kick = get_safe_input("do_kick (true/false): ") == "true";
        chip = get_safe_input("chip (true/false): ") == "true";
        forced = get_safe_input("forced (true/false): ") == "true";
        dribble_cclockwise = get_safe_input("dribble_cclockwise (true/false): ") == "true";
        dribble_vel = std::stoi(get_safe_input("dribble_vel (0-7): "));
    } else {
        std::cout << "Sending an example packet.\n";
        
        id = 1;
        robot_vel = 2000;
        w = 300;
        rot_cclockwise = true;
        w_vel = 1000;
        kick_force = 200;
        do_kick = true;
        chip = false;
        forced = true;
        dribble_cclockwise = true;
        dribble_vel = 5;
    }

    namespace bf = boost;

    #define FIELD(id) std::cout << bf::format("\t%-20s %-20s\n") % #id % get_pretty_value(id);

    std::cout << "Packet:\n";
    FIELD(id);
    FIELD(robot_vel);
    FIELD(w);
    FIELD(rot_cclockwise);
    FIELD(w_vel);
    FIELD(kick_force);
    FIELD(do_kick);
    FIELD(chip);
    FIELD(forced);
    FIELD(dribble_cclockwise);
    FIELD(dribble_vel);

    std::cout << "Creating message... ";

    auto possibleMsg = createRobotPacket(
            id,
            robot_vel,
            w,
            rot_cclockwise,
            w_vel,
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

    std::cout << "All params ok.\n";

    std::cout << "Message contents: \n";

    auto msg = *possibleMsg;

    for (const auto& byte : msg) {
        std::cout << "\t" << byteToBinary(byte) << "\t" << std::to_string(byte) << "\n";
    }

    if (!get_safe_input("Press enter to send the packet or type something to cancel...", false).empty()) {
        std::cout << "Sending message canceled. Aborting.\n";
        return 0;
    }

    std::cout << "Creating serial port...\n";

    boost::system::error_code errorCode;
    serialPort.open(output_file);
    switch (errorCode.value()) {
        case boost::system::errc::success:
            // Great!
            break;
        default:
            std::cout << "An error occurred while creating the file object. Have you passed the right path?\n";
            exit(1);
            break;
    }

    bool keepGoing = true;

    do {
        std::cout << "Writing bytes to files... ";

        serialPort.write_some(boost::asio::buffer(msg.data(), msg.size()));
        // TODO: @Hack base station crutches!
        serialPort.write_some(boost::asio::buffer(msg.data(), 1));

        std::cout << "Done.\n";

        if (get_safe_input("Check for ACK (Y/n): ", false) != "n") {
            uint8_t ackCode = 0;
            int receivedBytes = serialPort.read_some(boost::asio::buffer(&ackCode, 1));
            std::cout << "Received bytes: " << receivedBytes << "\n";
            std::cout << "The byte: " << std::to_string(ackCode) << "\n";
        }

        keepGoing = get_safe_input("Send again (Y/n): ", false) != "n";
    } while (keepGoing);

    return 0;
}

} // rtt

int main(int argc, char *argv[]) {
    std::vector<std::string> arguments(argv + 1, argv + argc);

    return rtt::main(arguments);
}

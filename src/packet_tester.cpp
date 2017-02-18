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

} // anonymous namespace

int main(const std::vector<std::string>& arguments) {
    ////////////////////////
    // Checking arguments //
    ////////////////////////
    
    if (arguments.size() < 1) {
        std::cout << "No output file specified as argument. Aborting.\n";
        return 1;
    }

    std::string output_file = arguments.at(0);

    std::cout << "Output file: " << output_file << "\n";

    /////////////////////
    // Creating packet //
    /////////////////////

    auto packetType = get_safe_input("Example packet or initialize packet by hand (EXAMPLE/manual/forward/backward/left/right)? ", "EXAMPLE");

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
    } else if (packetType == "forward" || packetType == "backward" || packetType == "left" || packetType == "right") {
        id = 7;
        robot_vel = 1000;
        ang = 0;
        rot_cclockwise = false;
        w = 0;
        kick_force = 0;
        do_kick = false;
        chip = false;
        forced = false;
        dribble_cclockwise = false;
        dribble_vel = 0;

        if (packetType == "forward") {
            ang = 128;
        } else if (packetType == "backward") {
            ang = 384;
        } else if (packetType == "left") {
            ang = 256;
        } else if (packetType == "right") {
            ang = 0;
        }
    }

    namespace bf = boost;

    /////////////////////
    // Printing packet //
    /////////////////////

    #define FIELD(id) std::cout << bf::format("\t%-20s %-20s\n") % #id % get_pretty_value(id);

    std::cout << "Packet:\n";
    FIELD(id);
    FIELD(robot_vel);
    FIELD(ang);
    FIELD(rot_cclockwise);
    FIELD(w);
    FIELD(kick_force);
    FIELD(do_kick);
    FIELD(chip);
    FIELD(forced);
    FIELD(dribble_cclockwise);
    FIELD(dribble_vel);

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

    std::cout << "All params ok.\n";

    ////////////////////////////
    // Printing binary packet //
    ////////////////////////////

    std::cout << "Message contents: \n";

    auto msg = *possibleMsg;

    for (const auto& byte : msg) {
        std::cout << "\t" << byteToBinary(byte) << "\t" << std::to_string(byte) << "\n";
    }

    //////////////////////////
    // Creating serial port //
    //////////////////////////

    if (!get_safe_input("Press enter to open the port or type something to cancel...").empty()) {
        std::cout << "Sending message canceled. Aborting.\n";
        return 0;
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

    ///////////////////////
    // Setting baud rate //
    ///////////////////////

    boost::asio::serial_port_base::baud_rate baudRate;
    serialPort.get_option(baudRate);
    std::cout << "Baud rate: " << baudRate.value() << "\n";
    
    std::cout << "Other standard baud rates: \n";
    for (auto br : baudRates) {
        std::cout << "    " << br << "\n";
    }
    std::cout << "Less standard baud rates: \n";
    for (auto br : nonStandardBaudRates) {
        std::cout << "    " << br << "\n";
    }

    auto userBaudRateStr = get_safe_input("Set baud rate (or empty to leave at standard): ");
    if (!userBaudRateStr.empty()) {
        try {
            int userBaudRate = std::stoi(userBaudRateStr);
            baudRate = boost::asio::serial_port_base::baud_rate(userBaudRate);
            boost::system::error_code ec;
            serialPort.set_option(baudRate, ec);

            if (ec != boost::system::errc::success) {
                std::cout << "Could not set baud rate. Error: " << ec << ". " << ec.message() << "\n";
            }

            serialPort.get_option(baudRate);
            std::cout << "Final baud rate: " << baudRate.value() << "\n";
        } catch (std::invalid_argument const & ex) {
            std::cout << "Invalid baudRate specified. Continuing with baudrate " << baudRate.value() << "\n";
        } catch (std::out_of_range const & ex) {
            std::cout << "Specified baud rate too large for an int. Continuing with baudrate " << baudRate.value() << "\n";
        }
    }

    if (get_safe_input("Benchmark (y/N)? ", "N") != "N") {
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

            if (checkForAck) {
                serialPort.read_some(boost::asio::buffer(ackCode, numBytes - 1));

                ackCode[numBytes - 1] = 0;

                std::string returnMessage((char*) &ackCode[0]);

                if (returnMessage[1] == '0') {
                    // std::cout << "X" << std::flush;
                    failCount++;
                }
                
                // std::cout << returnMessage << "\n";
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
    } else {
        ////////////////////////
        // Single packet test //
        ////////////////////////

        if (!get_safe_input("Press enter to continue type something to cancel...").empty()) {
            std::cout << "Sending message canceled. Aborting.\n";
            return 0;
        }

        bool keepGoing = true;

        do {
            std::cout << "Writing bytes to files... ";

            serialPort.write_some(boost::asio::buffer(msg.data(), msg.size()));
            // TODO: @Hack base station crutches! Pakcet length should be smaller
            serialPort.write_some(boost::asio::buffer(msg.data(), 1));

            std::cout << "Done.\n";

            if (get_safe_input("Check for ACK (Y/n): ", "Y") != "n") {
                int const numBytes = 3;
                uint8_t ackCode[numBytes];
                int receivedBytes = serialPort.read_some(boost::asio::buffer(ackCode, numBytes - 1));

                ackCode[numBytes - 1] = 0;

                std::cout << "Received bytes: " << receivedBytes << "\n";

                // std::cout << "The byte: " << std::to_string(ackCode[0]) << "\n";
                // std::cout << "The byte: " << std::to_string(ackCode[1]) << "\n";

                std::string returnMessage((char*) &ackCode[0]);
                std::cout << "The message: " << returnMessage << "\n";
            }

            keepGoing = get_safe_input("Send again (Y/n): ", "Y") != "n";
        } while (keepGoing);
    }

    return 0;
}

} // rtt

////////////////////
// Bitch-ass main //
////////////////////

int main(int argc, char *argv[]) {
    std::vector<std::string> arguments(argv + 1, argv + argc);

    return rtt::main(arguments);
}

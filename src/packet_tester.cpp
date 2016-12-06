#include <cstdint>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>

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

int main(const std::vector<std::string>& arguments) {
    if (arguments.size() < 1) {
        std::cout << "No output file specified as argument. Aborting.\n";
        return 1;
    }

    std::string output_file = arguments.at(0);

    std::cout << "Output file: " << output_file << "\n";

    bool manual = get_safe_input("Example packet or initialize packet by hand (EXAMPLE/manual)? ") == "manual";

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

    #define FIELD(id) std::cout << "\t" #id ": " << get_pretty_value(id) << "\n"

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

    if (!get_safe_input("Press enter to send the packet or type something to cancel...", false).empty()) {
        std::cout << "Sending message canceled. Aborting.\n";
        return 0;
    }

    auto msg = *possibleMsg;

    std::cout << "Creating file object...\n";

    std::ofstream fout(output_file, std::ios::binary | std::ios::out);

    if (!fout) {
        std::cout << "An error occurred while creating the file object. Have you passed the right path?\n";
        return 1;
    }

    std::cout << "Writing bytes to files... ";

    fout.write((char *) msg.data(), msg.size());
    
    std::cout << "Done.\n";

    return 0;
}

} // rtt

int main(int argc, char *argv[]) {
    std::vector<std::string> arguments(argv + 1, argv + argc);

    return rtt::main(arguments);
}

//
// Created by mrlukasbos on 8-3-19.
//

#ifndef ROBOTEAM_ROBOTHUB_UTILITIES_H
#define ROBOTEAM_ROBOTHUB_UTILITIES_H

#include <iostream>
#include <string>
#include "packing.h"

namespace rtt {
namespace robothub {
namespace utils {

enum class Mode {
    SERIAL,
    GRSIM,
    UNDEFINED
};

static int char2int(char input) {
    if (input>='0' && input<='9') return input-'0';
    if (input>='A' && input<='F') return input-'A'+10;
    if (input>='a' && input<='f') return input-'a'+10;

    // TODO look into this, (throw somehow cannot be used)
    throw std::invalid_argument("char2int : Invalid input string");
}

// This function assumes src to be a zero terminated sanitized string with an even number of [0-9a-f] characters, and target to be sufficiently large
static void hex2bin(const char* src, packed_robot_feedback& target) {
    int i;
    while (*src && src[1]) {
        target.at(i) = char2int(*src)*16+char2int(src[1]);
        src += 2;
        i++;
    }
}

static Mode stringToMode(const std::string& type) {
    if (type=="serial") {
        return Mode::SERIAL;
    }
    else if (type=="grsim") {
        return Mode::GRSIM;
    }
    else {
        return Mode::UNDEFINED;
    }
}

static std::string modeToString(Mode mode) {
    switch (mode) {
    case Mode::SERIAL:return "Serial";
    case Mode::GRSIM:return "Grsim";
    case Mode::UNDEFINED:return "Undefined";
    }
}

// Copy of getWorldBot() because I don't want to pull in tactics as a dependency.
// If this function is moved to utils, we can use that
static std::shared_ptr<proto::WorldRobot> getWorldBot(unsigned int id, bool ourTeam, const proto::World &world) {

    std::vector<proto::WorldRobot> robots;

    if (ourTeam) {
        robots = std::vector<proto::WorldRobot>( world.yellow().begin(),  world.yellow().end());
    } else {
        robots = std::vector<proto::WorldRobot>(world.blue().begin(), world.blue().end());

    }

    for (const auto &bot : robots) {
        if (bot.id() == id) {

            return std::make_shared<proto::WorldRobot>(bot);
        }
    }
    return nullptr;
}

static void printbits(packed_protocol_message byteArr) {
    for (int b = 0; b<12; b++) {

        std::cout << "    ";
        uint8_t byte = byteArr[b];
//            for (int i = 0; i < 8; i++) {
        for (int i = 7; i>=0; i--) {
            if ((byte & (1 << i))==(1 << i)) {
                std::cout << "1";
            } else {
                std::cout << "0";
            }
        }
        std::cout << std::endl;
    }
}

} // utils
} // robothub
} // rtt

#endif //ROBOTEAM_ROBOTHUB_UTILITIES_H

//
// Created by mrlukasbos on 8-3-19.
//

#ifndef ROBOTEAM_ROBOTHUB_UTILITIES_H
#define ROBOTEAM_ROBOTHUB_UTILITIES_H

#include <bitset>
#include <iostream>
#include <string>

#include "packing.h"
#include "roboteam_proto/World.pb.h"

namespace rtt {
namespace robothub {
namespace utils {

enum class Mode { SERIAL, GRSIM, UNDEFINED, SSL_SIMULATOR };

static int char2int(char input) {
    if (input >= '0' && input <= '9') return input - '0';
    if (input >= 'A' && input <= 'F') return input - 'A' + 10;
    if (input >= 'a' && input <= 'f') return input - 'a' + 10;

    // TODO look into this, (throw somehow cannot be used)
    /**
     * Strongly suggest to use std::optional then
     */
    throw std::invalid_argument("char2int : Invalid input string");
}

// This function assumes src to be a zero terminated sanitized string with an
// even number of [0-9a-f] characters, and target to be sufficiently large
[[maybe_unused]] static void hex2bin(const char* src, packed_robot_feedback& target) {
    int i{0};  // don't leave this uninitialized, the value is UB, incrementing
               // it could either be 1, 0, -500, 273613 or whatever else
    /**
     * be consistent
     * operator[] for pointers is essentiall *(ptr + i), so *src == src[0]
     */
    while (src[0] && src[1]) {
        target.at(i) = char2int(*src) * 16 + char2int(src[1]);
        src += 2;
        i++;
    }
}

/**
 * Inline implies inline, it does not guarantee it
 * However if a function marked constexpr then it's guaranteed to be inlined
 * Things that can be calculated compiletime **should** be actually calculated
 * compiletime Try stick to using constexpr as much as possible, you're using
 * C++17 so might aswel use modern features
 */
[[maybe_unused]] /* Why static?... inline instead */ static Mode stringToMode(const std::string& type) noexcept {
    if (type == "serial") {
        return Mode::SERIAL;
    } else if (type == "grsim") {
        return Mode::GRSIM;
    } else {
        return Mode::UNDEFINED;
    }
}

[[maybe_unused]] static std::string modeToString(Mode mode) noexcept {
    switch (mode) {
        case Mode::SERIAL:
            return "Serial";
        case Mode::GRSIM:
            return "Grsim";
        case Mode::UNDEFINED:
            return "Undefined";
    }
    // should never hit this:
    // Just to surpress compiler warnings
    return "";
}

// Copy of getWorldBot() because I don't want to pull in tactics as a
// dependency. If this function is moved to utils, we can use that
[[maybe_unused]] static std::shared_ptr<proto::WorldRobot> getWorldBot(unsigned int id, bool ourTeam, const proto::World& world) {
    /** Heavily inefficient, copying over all the robots :(
     * If this was C++20 I would've picked std::span, but for now just use
     * yellow() / blue()
     */
    // if (ourTeam) {
    //     robots = std::vector<roboteam_proto::WorldRobot>(
    //     world.yellow().begin(),  world.yellow().end());
    // } else {
    //     robots =
    //     std::vector<roboteam_proto::WorldRobot>(world.blue().begin(),
    //     world.blue().end());
    // }

    // Prevent a copy.

    auto& robots = ourTeam ? world.yellow() : world.blue();

    // https://en.cppreference.com/w/cpp/algorithm/find
    // Should do that instead, but whatever, doesn't really matter in terms of
    // performance
    for (const auto& bot : robots) {
        if (bot.id() == id) {
            return std::make_shared<proto::WorldRobot>(bot);
        }
    }
    return nullptr;
}

/**
 * Maybe unused surpresses compiler warnings, nothing major it's just better to
 * explicitly mark them unused
 */
[[maybe_unused]] static void printbits(packed_protocol_message const& byteArr) noexcept {
    // for (int b = 0; b<12; b++) {

    for (auto const bit : byteArr) {  // Litearlly def'd to std::array
                                      /**
                                       * std::array implements begin() and end(), hence this is easier
                                       */
        std::cout << "    " << std::bitset<8>(bit) << std::endl;
    }
    // for (int i = 0; i < 8; i++) {
    // for (int i = 7; i>=0; i--) {
    //     if ((byte & (1 << i))==(1 << i)) {
    //         std::cout << "1";
    //     } else {
    //         std::cout << "0";
    //     }
    // }
    // std::cout << std::endl;
}

}  // namespace utils
}  // namespace robothub
}  // namespace rtt

#endif  // ROBOTEAM_ROBOTHUB_UTILITIES_H

//
// Created by mrlukasbos on 8-3-19.
//

#ifndef ROBOTEAM_ROBOTHUB_UTILITIES_H
#define ROBOTEAM_ROBOTHUB_UTILITIES_H

#include <bitset>
#include <iostream>
#include <string>

#include "roboteam_proto/World.pb.h"

namespace rtt {
namespace robothub {
namespace utils {

enum class Mode { SERIAL, UNDEFINED };

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
    } else {
        return Mode::UNDEFINED;
    }
}

[[maybe_unused]] static std::string modeToString(Mode mode) noexcept {
    switch (mode) {
        case Mode::SERIAL:
            return "Serial";
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

}  // namespace utils
}  // namespace robothub
}  // namespace rtt

#endif  // ROBOTEAM_ROBOTHUB_UTILITIES_H

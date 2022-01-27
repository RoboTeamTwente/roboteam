#pragma once

#include <iostream>
#include <memory>
#include <string>

namespace rtt::robothub::utils {

enum class RobotHubMode { NEITHER, SIMULATOR, BASESTATION, BOTH };

enum class TeamColor { YELLOW, BLUE };

static std::string teamColorToString(TeamColor color) {
    switch (color) {
        case TeamColor::BLUE:
            return "BLUE";
        case TeamColor::YELLOW:
            return "YELLOW";
        default:
            return "UNDEFINED";
    }
}

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
[[maybe_unused]] /* Why static?... inline instead */ static RobotHubMode stringToMode(const std::string& type) noexcept {
    if (type == "Basestation") {
        return RobotHubMode::BASESTATION;
    } else if (type == "Simulator") {
        return RobotHubMode::SIMULATOR;
    } else if (type == "Both") {
        return RobotHubMode::BOTH;
    } else {
        return RobotHubMode::NEITHER;
    }
}

[[maybe_unused]] static std::string modeToString(RobotHubMode mode) noexcept {
    switch (mode) {
        case RobotHubMode::BASESTATION:
            return "Basestation";
        case RobotHubMode::SIMULATOR:
            return "Simulator";
        case RobotHubMode::BOTH:
            return "Both";
        default:
            return "Neither";
    }
}

}  // namespace rtt::robothub::utils

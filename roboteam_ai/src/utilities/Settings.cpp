#include "utilities/Settings.h"

#include <roboteam_utils/Print.h>
#include <utilities/IOManager.h>
#include <string_view>

namespace rtt {

std::atomic<int> Settings::id = Settings::PRIMARY_AI_ID;
std::atomic<bool> Settings::yellow = false;
std::atomic<bool> Settings::left = false;
std::atomic<Settings::RobotHubMode> Settings::robotHubMode = Settings::RobotHubMode::SIMULATOR;

void Settings::handleSettingsFromPrimaryAI(bool otherIsYellow, bool isLeft, RobotHubMode otherMode) {
    setYellow(!otherIsYellow);
    left = !isLeft;
    robotHubMode = otherMode;
}

int Settings::getId() { return id; }
bool Settings::isPrimaryAI() { return id == PRIMARY_AI_ID; }

void Settings::setId(int _id) { id = _id; }

bool Settings::isYellow() { return yellow; }

bool Settings::setYellow(bool isYellow) {
    if (ai::io::io.obtainTeamColorChannel(yellow)) {
        yellow = isYellow;
        return true;
    }

    // We could not obtain the necessary channel
    return false;
}

bool Settings::isLeft() { return left; }

void Settings::setLeft(bool _left) {
    if (isPrimaryAI()) {
        left = _left;
        return;
    }

    RTT_INFO("This secondary AI can not alter settings")
}

Settings::RobotHubMode Settings::getRobotHubMode() { return robotHubMode; }

bool Settings::setRobotHubMode(RobotHubMode mode) {
    // We can only switch mode if we are the primary AI
    if (isPrimaryAI()) {
        robotHubMode = mode;
        return true;
    }

    RTT_INFO("This secondary AI can not alter settings")
    return false;
}

std::string_view Settings::robotHubModeToString(RobotHubMode mode) {
    switch (mode) {
        case RobotHubMode::BASESTATION:
            return "Basestation";
        case RobotHubMode::SIMULATOR:
            return "Simulator";
        default:
            return "Unknown";
    }
}

}  // namespace rtt
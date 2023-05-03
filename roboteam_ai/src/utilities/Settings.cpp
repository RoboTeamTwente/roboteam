#include "utilities/Settings.h"

#include <roboteam_utils/Print.h>
#include <utilities/IOManager.h>
#include <string_view>
#include "roboteam_utils/RobotHubMode.h"

namespace rtt {

std::atomic<bool> Settings::primaryAI = false;
std::atomic<bool> Settings::yellow = false;
std::atomic<bool> Settings::left = false;
std::atomic<RobotHubMode> Settings::robotHubMode = RobotHubMode::SIMULATOR;

void Settings::handleSettingsFromPrimaryAI(const proto::Setting& settings) {
    setYellow(!settings.is_yellow());
    setLeft(!settings.is_left());
    robotHubMode = modeFromProto(settings.robot_hub_mode());
}

bool Settings::isPrimaryAI() { return primaryAI; }

void Settings::setPrimaryAI(bool value) {  primaryAI = value; }

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

RobotHubMode Settings::getRobotHubMode() { return robotHubMode; }

bool Settings::setRobotHubMode(RobotHubMode mode) {
    // We can only switch mode if we are the primary AI
    if (isPrimaryAI()) {
        robotHubMode = mode;
        return true;
    }

    RTT_INFO("This secondary AI can not alter settings")
    return false;
}

}  // namespace rtt
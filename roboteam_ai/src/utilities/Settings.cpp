#include <roboteam_utils/Print.h>
#include <utilities/IOManager.h>

#include <string_view>

#include "RobotHubMode.h"
#include "utilities/GameSettings.h"

namespace rtt {

std::atomic<bool> GameSettings::primaryAI = false;
std::atomic<bool> GameSettings::yellow = false;
std::atomic<bool> GameSettings::left = false;
std::atomic<net::RobotHubMode> GameSettings::robotHubMode = net::RobotHubMode::SIMULATOR;

void GameSettings::handleSettingsFromPrimaryAI(const proto::GameSettings& settings) {
    setYellow(!settings.is_yellow());
    setLeft(!settings.is_left());
    robotHubMode = net::modeFromProto(settings.robot_hub_mode());
}

bool GameSettings::isPrimaryAI() { return primaryAI; }

void GameSettings::setPrimaryAI(bool value) {  primaryAI = value; }

bool GameSettings::isYellow() { return yellow; }

bool GameSettings::setYellow(bool isYellow) {
    if (ai::io::io.obtainTeamColorChannel(isYellow)) {
        yellow = isYellow;
        return true;
    }

    // We could not obtain the necessary channel
    return false;
}

bool GameSettings::isLeft() { return left; }

void GameSettings::setLeft(bool _left) {
    if (isPrimaryAI()) {
        left = _left;
        return;
    }

    RTT_INFO("This secondary AI can not alter settings")
}

net::RobotHubMode GameSettings::getRobotHubMode() { return robotHubMode; }

bool GameSettings::setRobotHubMode(net::RobotHubMode mode) {
    // We can only switch mode if we are the primary AI
    if (isPrimaryAI()) {
        robotHubMode = mode;
        return true;
    }

    RTT_INFO("This secondary AI can not alter settings")
    return false;
}

}  // namespace rtt
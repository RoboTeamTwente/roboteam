#include <roboteam_utils/Print.h>
#include <utilities/IOManager.h>

#include <string_view>

#include "RobotHubMode.h"
#include "utilities/GameSettings.h"

namespace rtt {

std::atomic<bool> GameSettings::has_changes = false;
std::atomic<bool> GameSettings::primary_AI = false;
std::atomic<bool> GameSettings::yellow = false;
std::atomic<bool> GameSettings::left = false;
std::atomic<net::RobotHubMode> GameSettings::robothub_mode = net::RobotHubMode::SIMULATOR;

bool GameSettings::hasChanges(){
    bool has_changes_copy = has_changes;
    has_changes = false;
    return has_changes_copy;
}

void GameSettings::copyFrom(const proto::GameSettings& settings) {
    setYellow(!settings.is_yellow());
    setLeft(!settings.is_left());
    setRobotHubMode(net::robotHubModeFromProto(settings.robot_hub_mode()));
}

bool GameSettings::isPrimaryAI() { return primary_AI; }

void GameSettings::setPrimaryAI(bool is_primary_AI) { 
    if(primary_AI != is_primary_AI){
        has_changes = true;
        primary_AI = is_primary_AI;
        RTT_INFO("Setting primary AI to ", primary_AI ? "true" : "false");
    }
}

bool GameSettings::isYellow() { return yellow; }

bool GameSettings::setYellow(bool is_yellow) {
    // Try to obtain the necessary channel
    if (ai::io::io.obtainTeamColorChannel(is_yellow)) {
        // If the color has changed, we need to update the hasChanges flag
        if (yellow != is_yellow) {
            RTT_INFO("Setting team color to ", is_yellow ? "yellow" : "blue");
            has_changes = true;
            yellow = is_yellow;
        }
        return true;
    }

    // We could not obtain the necessary channel
    return false;
}

bool GameSettings::isLeft() { return left; }

void GameSettings::setLeft(bool is_left) {
    if(left != is_left){
        has_changes = true;
        left = is_left;
        RTT_INFO("Setting left to ", left ? "left" : "right");
    }
}

net::RobotHubMode GameSettings::getRobotHubMode() { return robothub_mode; }

void GameSettings::setRobotHubMode(net::RobotHubMode mode) {
    if(robothub_mode != mode){
        has_changes = true;
        robothub_mode = mode;
        RTT_INFO("Setting robot hub mode to ", robotHubModeToString(mode));
    }
}

}  // namespace rtt
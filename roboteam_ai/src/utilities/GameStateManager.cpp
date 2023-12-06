#include "utilities/GameStateManager.hpp"

#include <roboteam_utils/Print.h>

#include "interface/api/Output.h"
#include "utilities/GameSettings.h"
#include "utilities/RuntimeConfig.h"
#include "utilities/StrategyManager.h"
#include "world/World.hpp"

namespace rtt::ai {

proto::SSL_Referee GameStateManager::refMsg;
StrategyManager GameStateManager::strategymanager;
std::mutex GameStateManager::refMsgLock;

proto::SSL_Referee GameStateManager::getRefereeData() {
    std::lock_guard<std::mutex> lock(refMsgLock);
    return GameStateManager::refMsg;
}

RefCommand GameStateManager::getCommandFromRefMsg(proto::SSL_Referee_Command command, bool isYellow) {
    switch (command) {
        case proto::SSL_Referee_Command_HALT:
            return RefCommand::HALT;
        case proto::SSL_Referee_Command_STOP:
            return RefCommand::STOP;
        case proto::SSL_Referee_Command_NORMAL_START:
            return RefCommand::NORMAL_START;
        case proto::SSL_Referee_Command_FORCE_START:
            return RefCommand::FORCED_START;
        case proto::SSL_Referee_Command_PREPARE_KICKOFF_YELLOW:
            return isYellow ? RefCommand::PREPARE_KICKOFF_US : RefCommand::PREPARE_KICKOFF_THEM;
        case proto::SSL_Referee_Command_PREPARE_KICKOFF_BLUE:
            return isYellow ? RefCommand::PREPARE_KICKOFF_THEM : RefCommand::PREPARE_KICKOFF_US;
        case proto::SSL_Referee_Command_PREPARE_PENALTY_YELLOW:
            return isYellow ? RefCommand::PREPARE_PENALTY_US : RefCommand::PREPARE_PENALTY_THEM;
        case proto::SSL_Referee_Command_PREPARE_PENALTY_BLUE:
            return isYellow ? RefCommand::PREPARE_PENALTY_THEM : RefCommand::PREPARE_PENALTY_US;
        case proto::SSL_Referee_Command_DIRECT_FREE_YELLOW:
            return isYellow ? RefCommand::DIRECT_FREE_US : RefCommand::DIRECT_FREE_THEM;
        case proto::SSL_Referee_Command_DIRECT_FREE_BLUE:
            return isYellow ? RefCommand::DIRECT_FREE_THEM : RefCommand::DIRECT_FREE_US;
        case proto::SSL_Referee_Command_INDIRECT_FREE_YELLOW:
            return isYellow ? RefCommand::INDIRECT_FREE_US : RefCommand::INDIRECT_FREE_THEM;
        case proto::SSL_Referee_Command_INDIRECT_FREE_BLUE:
            return isYellow ? RefCommand::INDIRECT_FREE_THEM : RefCommand::INDIRECT_FREE_US;
        case proto::SSL_Referee_Command_TIMEOUT_YELLOW:
            return isYellow ? RefCommand::TIMEOUT_US : RefCommand::TIMEOUT_THEM;
        case proto::SSL_Referee_Command_TIMEOUT_BLUE:
            return isYellow ? RefCommand::TIMEOUT_THEM : RefCommand::TIMEOUT_US;
        case proto::SSL_Referee_Command_BALL_PLACEMENT_YELLOW:
            return isYellow ? RefCommand::BALL_PLACEMENT_US : RefCommand::BALL_PLACEMENT_THEM;
        case proto::SSL_Referee_Command_BALL_PLACEMENT_BLUE:
            return isYellow ? RefCommand::BALL_PLACEMENT_THEM : RefCommand::BALL_PLACEMENT_US;
        default:
            RTT_ERROR("Unknown refstate, halting all robots for safety!")
            return RefCommand::HALT;
    }
}

void GameStateManager::setRefereeData(proto::SSL_Referee refMsg, const rtt::world::World* data) {
    std::lock_guard<std::mutex> lock(refMsgLock);
    GameStateManager::refMsg = refMsg;
    bool isYellow = GameSettings::isYellow();
    RefCommand cmd = getCommandFromRefMsg(refMsg.command(), isYellow);

    auto stage = refMsg.stage();
    auto world = data->getWorld();
    if (world.has_value()) {
        strategymanager.setCurrentRefGameState(cmd, stage, world->getBall());
    }
}

// Initialize static variables
GameState GameStateManager::getCurrentGameState() {
    GameState newGameState;
    if (RuntimeConfig::useReferee) {
        newGameState = static_cast<GameState>(strategymanager.getCurrentRefGameState());

        if (GameSettings::isYellow()) {
            newGameState.keeperId = getRefereeData().yellow().goalkeeper();
            newGameState.maxAllowedRobots = getRefereeData().yellow().max_allowed_bots();
        } else {
            newGameState.keeperId = getRefereeData().blue().goalkeeper();
            newGameState.maxAllowedRobots = getRefereeData().blue().max_allowed_bots();
        }

        // TODO: FIX for the new config system
        // if there is a ref we set the interface gamestate to these values as well
        // this makes sure that when we stop using the referee we don't return to an unknown state,
        // // so now we keep the same.
        interface::Output::setInterfaceGameState(newGameState);
    } else {
        newGameState = interface::Output::getInterfaceGameState();
    }
    return newGameState;
}

void GameStateManager::forceNewGameState(RefCommand cmd) {
    RTT_INFO("Forcing new refstate!")

    // overwrite both the interface and the strategy manager.
    interface::Output::setInterfaceGameState(strategymanager.getRefGameStateForRefCommand(cmd));
    strategymanager.forceCurrentRefGameState(cmd);
}

Vector2 GameStateManager::getRefereeDesignatedPosition() {
    auto designatedPos = rtt::ai::GameStateManager::getRefereeData().designated_position();
    return Vector2(designatedPos.x() / 1000, designatedPos.y() / 1000);
}

void GameStateManager::updateInterfaceGameState(const char* name) {
    static const std::map<std::string, std::pair<std::string, rtt::ai::RuleSet>> nameToGameState = {
        {"Aggressive Stop Formation", {"stop", Constants::RULESET_STOP()}},
        {"Defensive Stop Formation", {"stop", Constants::RULESET_STOP()}},
        {"Ball Placement Us", {"ball_placement_us", Constants::RULESET_BALLPLACEMENT_US()}},
        {"Ball Placement Them", {"ball_placement_them", Constants::RULESET_BALLPLACEMENT_THEM()}},
        {"Halt", {"halt", Constants::RULESET_HALT()}},
        {"Free Kick Them", {"free_kick_them", Constants::RULESET_STOP()}},
        {"Free Kick Us At Goal", {"free_kick_us", Constants::RULESET_DEFAULT()}},
        {"Free Kick Us Pass", {"free_kick_us", Constants::RULESET_DEFAULT()}},
        {"Kick Off Us Prepare", {"kickoff_us_prepare", Constants::RULESET_KICKOFF()}},
        {"Kick Off Them Prepare", {"kickoff_them_prepare", Constants::RULESET_KICKOFF()}},
        {"Kick Off Us", {"kickoff_us", Constants::RULESET_DEFAULT()}},
        {"Kick Off Them", {"kickoff_them", Constants::RULESET_DEFAULT()}},
        {"Penalty Us Prepare", {"penalty_us_prepare", Constants::RULESET_DEFAULT()}},
        {"Penalty Them Prepare", {"penalty_them_prepare", Constants::RULESET_DEFAULT()}},
        {"Penalty Us", {"penalty_us", Constants::RULESET_DEFAULT()}},
        {"Penalty Them", {"penalty_them", Constants::RULESET_DEFAULT()}},
        {"Time Out", {"time_out", Constants::RULESET_DEFAULT()}},
        {"Defensive Stop Formation", {"stop", Constants::RULESET_STOP()}},
        {"Aggressive Stop Formation", {"stop", Constants::RULESET_STOP()}},        
    };

    auto it = nameToGameState.find(name);
    if (it != nameToGameState.end()) {
        interface::Output::setInterfaceGameState(GameState(it->second.first, it->second.second));
    } else {
        RTT_WARNING("Play has been selected for which no ruleset is found!");
        interface::Output::setInterfaceGameState(GameState("normal_play", Constants::RULESET_DEFAULT()));
    }
}
}  // namespace rtt::ai

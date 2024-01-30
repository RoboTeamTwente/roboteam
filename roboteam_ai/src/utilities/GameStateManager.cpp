#include "utilities/GameStateManager.hpp"

#include <roboteam_utils/Print.h>

#include "interface/api/Output.h"
#include "utilities/GameSettings.h"
#include "utilities/RuntimeConfig.h"
#include "utilities/StrategyManager.h"
#include "world/World.hpp"

namespace rtt::ai {
int GameState::cardId = -1;

proto::Referee GameStateManager::refMsg;
StrategyManager GameStateManager::strategymanager;
std::mutex GameStateManager::refMsgLock;

proto::Referee GameStateManager::getRefereeData() {
    std::lock_guard<std::mutex> lock(refMsgLock);
    return GameStateManager::refMsg;
}

RefCommand GameStateManager::getCommandFromRefMsg(proto::Referee_Command command, bool isYellow) {
    switch (command) {
        case proto::Referee_Command_HALT:
            return RefCommand::HALT;
        case proto::Referee_Command_STOP:
            return RefCommand::STOP;
        case proto::Referee_Command_NORMAL_START:
            return RefCommand::NORMAL_START;
        case proto::Referee_Command_FORCE_START:
            return RefCommand::FORCED_START;
        case proto::Referee_Command_PREPARE_KICKOFF_YELLOW:
            return isYellow ? RefCommand::PREPARE_KICKOFF_US : RefCommand::PREPARE_KICKOFF_THEM;
        case proto::Referee_Command_PREPARE_KICKOFF_BLUE:
            return isYellow ? RefCommand::PREPARE_KICKOFF_THEM : RefCommand::PREPARE_KICKOFF_US;
        case proto::Referee_Command_PREPARE_PENALTY_YELLOW:
            return isYellow ? RefCommand::PREPARE_PENALTY_US : RefCommand::PREPARE_PENALTY_THEM;
        case proto::Referee_Command_PREPARE_PENALTY_BLUE:
            return isYellow ? RefCommand::PREPARE_PENALTY_THEM : RefCommand::PREPARE_PENALTY_US;
        case proto::Referee_Command_DIRECT_FREE_YELLOW:
            return isYellow ? RefCommand::DIRECT_FREE_US : RefCommand::DIRECT_FREE_THEM;
        case proto::Referee_Command_DIRECT_FREE_BLUE:
            return isYellow ? RefCommand::DIRECT_FREE_THEM : RefCommand::DIRECT_FREE_US;
        case proto::Referee_Command_TIMEOUT_YELLOW:
            return isYellow ? RefCommand::TIMEOUT_US : RefCommand::TIMEOUT_THEM;
        case proto::Referee_Command_TIMEOUT_BLUE:
            return isYellow ? RefCommand::TIMEOUT_THEM : RefCommand::TIMEOUT_US;
        case proto::Referee_Command_BALL_PLACEMENT_YELLOW:
            return isYellow ? RefCommand::BALL_PLACEMENT_US : RefCommand::BALL_PLACEMENT_THEM;
        case proto::Referee_Command_BALL_PLACEMENT_BLUE:
            return isYellow ? RefCommand::BALL_PLACEMENT_THEM : RefCommand::BALL_PLACEMENT_US;
        default:
            RTT_ERROR("Unknown refstate, halting all robots for safety!")
            return RefCommand::HALT;
    }
}

void GameStateManager::setRefereeData(proto::Referee refMsg, const rtt::world::World* data) {
    std::lock_guard<std::mutex> lock(refMsgLock);
    GameStateManager::refMsg = refMsg;
    bool isYellow = GameSettings::isYellow();
    RefCommand cmd = getCommandFromRefMsg(refMsg.command(), isYellow);

    auto stage = refMsg.stage();
    auto world = data->getWorld();
    if (world.has_value()) {
        strategymanager.setCurrentGameState(cmd, stage, world->getBall());
    }
}

// Initialize static variables
GameState GameStateManager::getCurrentGameState() {
    GameState newGameState;
    if (RuntimeConfig::useReferee) {
        newGameState = strategymanager.getCurrentGameState();

        if (GameSettings::isYellow()) {
            newGameState.keeperId = getRefereeData().yellow().goalkeeper();
            newGameState.maxAllowedRobots = getRefereeData().yellow().max_allowed_bots();
        } else {
            newGameState.keeperId = getRefereeData().blue().goalkeeper();
            newGameState.maxAllowedRobots = getRefereeData().blue().max_allowed_bots();
        }

        interface::Output::setInterfaceGameState(newGameState);
    } else {
        newGameState = interface::Output::getInterfaceGameState();
    }
    return newGameState;
}

void GameStateManager::forceNewGameState(RefCommand cmd) {
    RTT_INFO("Forcing new refstate!")
    interface::Output::setInterfaceGameState(strategymanager.getGameStateForRefCommand(cmd));
    strategymanager.forceCurrentGameState(cmd);
}

Vector2 GameStateManager::getRefereeDesignatedPosition() {
    auto designatedPos = rtt::ai::GameStateManager::getRefereeData().designated_position();
    return Vector2(designatedPos.x() / 1000, designatedPos.y() / 1000);
}

void GameStateManager::updateInterfaceGameState(const char* name) {
    static const std::map<std::string, std::pair<RefCommand, rtt::ai::RuleSet>> nameToGameState = {
        {"Aggressive Stop Formation", {RefCommand::STOP, Constants::RULESET_STOP()}},
        {"Defensive Stop Formation", {RefCommand::STOP, Constants::RULESET_STOP()}},
        {"Ball Placement Us", {RefCommand::BALL_PLACEMENT_US, Constants::RULESET_STOP()}},
        {"Ball Placement Them", {RefCommand::BALL_PLACEMENT_THEM, Constants::RULESET_STOP()}},
        {"Halt", {RefCommand::HALT, Constants::RULESET_HALT()}},
        {"Free Kick Them", {RefCommand::DIRECT_FREE_THEM, Constants::RULESET_DEFAULT()}},
        {"Free Kick Us At Goal", {RefCommand::DIRECT_FREE_US, Constants::RULESET_DEFAULT()}},
        {"Free Kick Us Pass", {RefCommand::DIRECT_FREE_US, Constants::RULESET_DEFAULT()}},
        {"Kick Off Us Prepare", {RefCommand::PREPARE_KICKOFF_US, Constants::RULESET_STOP()}},
        {"Kick Off Them Prepare", {RefCommand::PREPARE_KICKOFF_THEM, Constants::RULESET_STOP()}},
        {"Kick Off Us", {RefCommand::KICKOFF_US, Constants::RULESET_DEFAULT()}},
        {"Kick Off Them", {RefCommand::KICKOFF_THEM, Constants::RULESET_DEFAULT()}},
        {"Penalty Us Prepare", {RefCommand::PREPARE_PENALTY_US, Constants::RULESET_STOP()}},
        {"Penalty Them Prepare", {RefCommand::PREPARE_PENALTY_THEM, Constants::RULESET_STOP()}},
        {"Penalty Us", {RefCommand::PENALTY_US, Constants::RULESET_DEFAULT()}},
        {"Penalty Them", {RefCommand::PENALTY_THEM, Constants::RULESET_DEFAULT()}},
        {"Time Out", {RefCommand::TIMEOUT_US, Constants::RULESET_HALT()}},
        {"Attacking Pass", {RefCommand::NORMAL_START, Constants::RULESET_DEFAULT()}},
        {"Attack", {RefCommand::NORMAL_START, Constants::RULESET_DEFAULT()}},
        {"Defend Shot", {RefCommand::NORMAL_START, Constants::RULESET_DEFAULT()}},
        {"Defend Pass", {RefCommand::NORMAL_START, Constants::RULESET_DEFAULT()}},
        {"Keeper Kick Ball", {RefCommand::NORMAL_START, Constants::RULESET_DEFAULT()}},
    };

    auto it = nameToGameState.find(name);
    if (it != nameToGameState.end()) {
        interface::Output::setInterfaceGameState(GameState(it->second.first, it->second.second));
    } else {
        RTT_WARNING("Play has been selected for which no ruleset is found!");
        interface::Output::setInterfaceGameState(GameState(RefCommand::HALT, Constants::RULESET_HALT()));
    }
}
}  // namespace rtt::ai

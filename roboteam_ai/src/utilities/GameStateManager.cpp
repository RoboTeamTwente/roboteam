#include "utilities/GameStateManager.hpp"

#include <roboteam_utils/Print.h>

#include "interface/api/Output.h"
#include "utilities/GameSettings.h"
#include "utilities/RuntimeConfig.h"
#include "utilities/StrategyManager.h"
#include "world/World.hpp"

namespace rtt::ai {
int GameState::cardId = -1;
double GameState::timeLeft = 1;

proto::Referee_TeamInfo GameStateManager::yellowTeam;
proto::Referee_TeamInfo GameStateManager::blueTeam;
proto::Referee_Point GameStateManager::refereeDesignatedPosition;
StrategyManager GameStateManager::strategymanager;
std::mutex GameStateManager::refMsgLock;

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
    {
        std::lock_guard<std::mutex> lock(refMsgLock);
        GameStateManager::yellowTeam = refMsg.yellow();
        GameStateManager::blueTeam = refMsg.blue();
        GameStateManager::refereeDesignatedPosition = refMsg.designated_position();
        GameState::timeLeft = static_cast<double>(refMsg.current_action_time_remaining()) / 1000000;
    }
    bool isYellow = GameSettings::isYellow();
    RefCommand command = getCommandFromRefMsg(refMsg.command(), isYellow);
    RefCommand nextCommand = refMsg.has_next_command() ? getCommandFromRefMsg(refMsg.next_command(), isYellow) : RefCommand::UNDEFINED;

    auto world = data->getWorld();
    if (world.has_value()) {
        strategymanager.setCurrentGameState(command, nextCommand, world->getBall());
    } else {
        RTT_INFO("No world when setting Game State, ignoring ball")
        strategymanager.setCurrentGameState(command, nextCommand, std::nullopt);
    }
}

// Initialize static variables
GameState GameStateManager::getCurrentGameState() {
    GameState newGameState;
    if (RuntimeConfig::useReferee) {
        newGameState = strategymanager.getCurrentGameState();
        {
            std::lock_guard<std::mutex> lock(refMsgLock);
            if (GameSettings::isYellow()) {
                newGameState.keeperId = GameStateManager::yellowTeam.goalkeeper();
                newGameState.maxAllowedRobots = GameStateManager::yellowTeam.max_allowed_bots();
            } else {
                newGameState.keeperId = GameStateManager::blueTeam.goalkeeper();
                newGameState.maxAllowedRobots = GameStateManager::blueTeam.max_allowed_bots();
            }
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
    proto::Referee_Point designatedPos;
    {
        std::lock_guard<std::mutex> lock(refMsgLock);
        designatedPos = GameStateManager::refereeDesignatedPosition;
    }
    return Vector2(designatedPos.x() / 1000, designatedPos.y() / 1000);
}

void GameStateManager::updateInterfaceGameState(const char* name) {
    static const std::map<std::string, std::pair<RefCommand, rtt::ai::RuleSet>> nameToGameState = {
        {"Stop Formation", {RefCommand::STOP, Constants::RULESET_STOP()}},
        {"Prepare Forced Start", {RefCommand::STOP, Constants::RULESET_STOP()}},
        {"Ball Placement Us Free Kick", {RefCommand::BALL_PLACEMENT_US_DIRECT, Constants::RULESET_DEFAULT()}},
        {"Ball Placement Us Force Start", {RefCommand::BALL_PLACEMENT_US, Constants::RULESET_DEFAULT()}},
        {"Ball Placement Them", {RefCommand::BALL_PLACEMENT_THEM, Constants::RULESET_DEFAULT()}},
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

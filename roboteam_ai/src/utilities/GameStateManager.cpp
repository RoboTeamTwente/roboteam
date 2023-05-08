#include "utilities/GameStateManager.hpp"

#include <roboteam_utils/Print.h>

#include <string>

#include "interface_api/RuntimeConfig.h"
#include "utilities/GameSettings.h"
#include "utilities/GameState.h"
#include "utilities/RefCommand.h"
#include "utilities/RefGameState.h"
#include "utilities/StrategyManager.h"
#include "world/World.hpp"

namespace rtt::ai {

proto::SSL_Referee GameStateManager::refMsg;
std::mutex GameStateManager::lock;

RefGameState GameStateManager::currentRefGameState = {RefCommand::UNDEFINED, "halt", "halt"};
GameState GameStateManager::currentInterfaceGameState = {"halt", "halt"};

const std::unordered_map<std::string_view, GameState> GameStateManager::playNameGameStateMapping = {
    {"Ball Placement Us", {"ball_placement_us", "ballplacement_us"}},
    {"Halt", {"halt", "halt"}},
    {"Free Kick Them", {"free_kick_them", "stop"}},
    {"Free Kick Us At Goal", {"free_kick_us", "default"}},
    {"Free Kick Us Pass", {"free_kick_us", "default"}},
    {"Ball Placement Them", {"ball_placement_them", "ballplacement_them"}},
    {"Kick Off Them Prepare", {"kickoff_them_prepare", "kickoff"}},
    {"Kick Off Us Prepare", {"kickoff_us_prepare", "kickoff"}},
    {"Kick Off Them", {"kickoff_them", "default"}},
    {"Kick Off Us", {"kickoff_us", "default"}},
    {"Penalty Them Prepare", {"penalty_them_prepare", "default"}},
    {"Penalty Us Prepare", {"penalty_us_prepare", "default"}},
    {"Penalty Them", {"penalty_them", "default"}},
    {"Penalty Us", {"penalty_us", "default"}},
    {"Time Out", {"time_out", "default"}},
    {"Defensive Stop Formation", {"stop", "stop"}},
    {"Aggressive Stop Formation", {"stop", "stop"}},
};

const std::unordered_map<RefCommand, RefGameState> GameStateManager::refCommandToGameStateMapping = {
    {RefCommand::UNDEFINED, RefGameState(RefCommand::UNDEFINED, "halt", "halt")},
    {RefCommand::NORMAL_START, RefGameState(RefCommand::NORMAL_START, "normal_play", "default")},
    {RefCommand::FORCED_START, RefGameState(RefCommand::FORCED_START, "normal_play", "default")},
    {RefCommand::HALT, RefGameState(RefCommand::HALT, "halt", "halt")},
    {RefCommand::STOP, RefGameState(RefCommand::STOP, "ball_placement_them", "ballplacement_them")},
    {RefCommand::TIMEOUT_US, RefGameState(RefCommand::TIMEOUT_US, "halt", "halt")},
    {RefCommand::TIMEOUT_THEM, RefGameState(RefCommand::TIMEOUT_THEM, "halt", "halt")},
    {RefCommand::GOAL_US, RefGameState(RefCommand::GOAL_US, "kickoff_them_prepare", "default")},
    {RefCommand::GOAL_THEM, RefGameState(RefCommand::GOAL_THEM, "kickoff_us_prepare", "default")},
    {RefCommand::BALL_PLACEMENT_US, RefGameState(RefCommand::BALL_PLACEMENT_US, "ball_placement_us", "ballplacement_us")},
    {RefCommand::BALL_PLACEMENT_THEM, RefGameState(RefCommand::BALL_PLACEMENT_THEM, "ball_placement_them", "ballplacement_them")},
    {RefCommand::DIRECT_FREE_US, RefGameState(RefCommand::DIRECT_FREE_US, "free_kick_us", "default")},
    {RefCommand::DIRECT_FREE_THEM, RefGameState(RefCommand::DIRECT_FREE_THEM, "free_kick_them", "stop")},
    {RefCommand::INDIRECT_FREE_US, RefGameState(RefCommand::INDIRECT_FREE_US, "free_kick_us", "default")},
    {RefCommand::INDIRECT_FREE_THEM, RefGameState(RefCommand::INDIRECT_FREE_THEM, "free_kick_them", "stop")},
    {RefCommand::PREPARE_KICKOFF_US, RefGameState(RefCommand::PREPARE_KICKOFF_US, "kickoff_us_prepare", "kickoff", false, RefCommand::DO_KICKOFF)},
    {RefCommand::PREPARE_KICKOFF_THEM, RefGameState(RefCommand::PREPARE_KICKOFF_THEM, "kickoff_them_prepare", "kickoff", false, RefCommand::DEFEND_KICKOFF)},
    {RefCommand::PREPARE_PENALTY_US, RefGameState(RefCommand::PREPARE_PENALTY_US, "penalty_us_prepare", "default", false, RefCommand::DO_PENALTY)},
    {RefCommand::PREPARE_PENALTY_THEM, RefGameState(RefCommand::PREPARE_PENALTY_THEM, "penalty_them_prepare", "default", false, RefCommand::DEFEND_PENALTY)},
    {RefCommand::DO_KICKOFF, RefGameState(RefCommand::DO_KICKOFF, "kickoff_us", "default", true)},
    {RefCommand::DEFEND_KICKOFF, RefGameState(RefCommand::DEFEND_KICKOFF, "kickoff_them", "default", true)},
    {RefCommand::DO_PENALTY, RefGameState(RefCommand::DO_PENALTY, "penalty_us", "default", true)},
    {RefCommand::DEFEND_PENALTY, RefGameState(RefCommand::DEFEND_PENALTY, "penalty_them", "default", true)},
    {RefCommand::DO_SHOOTOUT, RefGameState(RefCommand::DO_SHOOTOUT, "time_out", "default", true)},
    {RefCommand::DEFEND_SHOOTOUT, RefGameState(RefCommand::DEFEND_SHOOTOUT, "time_out", "default", true)},
    {RefCommand::PREPARE_SHOOTOUT_US, RefGameState(RefCommand::PREPARE_SHOOTOUT_US, "penalty_us_prepare", "default", false, RefCommand::DO_PENALTY)},
    {RefCommand::PREPARE_SHOOTOUT_THEM, RefGameState(RefCommand::PREPARE_SHOOTOUT_THEM, "penalty_them_prepare", "default", false, RefCommand::DEFEND_SHOOTOUT)},
    {RefCommand::PRE_HALF, RefGameState(RefCommand::PRE_HALF, "formation_pre_half", "default", false)}};

proto::SSL_Referee GameStateManager::getRefereeData() {
    std::lock_guard<std::mutex> _(lock);
    return GameStateManager::refMsg;
}

void GameStateManager::setGameStateFromReferee(proto::SSL_Referee refMsg, std::optional<world::view::WorldDataView> world) {
    std::lock_guard<std::mutex> _(lock);
    GameStateManager::refMsg = refMsg;

    if (!world.has_value()) {
        RTT_ERROR("No world available to set game state from referee data")
        return;
    }

    auto newRefCmd = rtt::sslRefCmdToRefCmd(refMsg.command(), GameSettings::isYellow());
    currentRefGameState = findNextRefGameState(refMsg.stage(), newRefCmd, world.value()->getBall());
    currentRefGameState.keeperId = static_cast<int>((GameSettings::isYellow() ? refMsg.yellow() : refMsg.blue()).goalkeeper()); // TODO: Will this work? IE is setGameStateFromReferee called everytime the ref sends a new message?

    // TODO: Forward changes to the interface
}

void GameStateManager::setGameStateFromInterface(std::string_view playName, std::optional<std::string_view> ruleset, std::optional<int> keeperId) {
    std::lock_guard<std::mutex> _(lock);

    auto iter = playNameGameStateMapping.find(playName);
    auto gameState = GameState("normal_play", "default");
    if (iter != playNameGameStateMapping.end()) {
        gameState = iter->second;
    }

    if (ruleset.has_value()) {
        gameState.ruleSetName = ruleset.value();
    }

    if (keeperId.has_value()) {
        gameState.keeperId = keeperId.value();
    }

    currentInterfaceGameState = std::move(gameState);
}

GameState GameStateManager::getCurrentGameState() {
    std::lock_guard<std::mutex> _(lock);
    return new_interface::RuntimeConfig::useReferee ? static_cast<GameState>(currentRefGameState) : currentInterfaceGameState;
}

void GameStateManager::forceNewGameState(RefCommand cmd, std::optional<rtt::world::view::BallView> ball) {
    RTT_INFO("Forcing new GameState!")
    std::lock_guard<std::mutex> _(lock);

    // overwrite both the interface and the strategy manager.
    currentInterfaceGameState = static_cast<GameState>(refCommandToGameStateMapping.at(cmd));  // This is strange, but basically we force the interface to be in the same state
                                                                                               // as the referee
    currentRefGameState = refCommandToGameStateMapping.at(cmd);

    // TODO: Forward changes to the interface
}

Vector2 GameStateManager::getRefereeDesignatedPosition() {
    std::lock_guard<std::mutex> _(lock);
    const auto& designatedPos = refMsg.designated_position();
    return {designatedPos.x() / 1000, designatedPos.y() / 1000};
}

RefGameState GameStateManager::findNextRefGameState(const proto::SSL_Referee_Stage& stage, RefCommand newRefCmd, std::optional<world::view::BallView> ballView) {
    // if the stage is shootout, we interpret penalty commands as shootOut penalty commands
    if (stage == proto::SSL_Referee_Stage_PENALTY_SHOOTOUT) {
        if (newRefCmd == RefCommand::PREPARE_PENALTY_US) {
            newRefCmd = RefCommand::PREPARE_SHOOTOUT_US;
        } else if (newRefCmd == RefCommand::PREPARE_PENALTY_THEM) {
            newRefCmd = RefCommand::PREPARE_SHOOTOUT_THEM;
        }
    }

    // If game is in pre start stage and the game is in stop state
    if (newRefCmd == RefCommand::STOP && (stage == proto::SSL_Referee_Stage_NORMAL_FIRST_HALF_PRE || stage == proto::SSL_Referee_Stage_NORMAL_SECOND_HALF_PRE ||
                                          stage == proto::SSL_Referee_Stage_EXTRA_FIRST_HALF_PRE || stage == proto::SSL_Referee_Stage_EXTRA_SECOND_HALF_PRE)) {
        newRefCmd = RefCommand::PRE_HALF;
    }

    // If the ball has been kicked during kickoff or a free kick, continue with NORMAL_START
    bool isBallMoving = ballView.has_value() && ballView.value()->velocity.length() > stp::control_constants::BALL_IS_MOVING_SLOW_LIMIT;
    if (isBallMoving && (currentRefGameState.commandId == RefCommand::DIRECT_FREE_THEM || currentRefGameState.commandId == RefCommand::DIRECT_FREE_US ||
                         currentRefGameState.commandId == RefCommand::DO_KICKOFF || currentRefGameState.commandId == RefCommand::DEFEND_KICKOFF ||
                         currentRefGameState.commandId == RefCommand::INDIRECT_FREE_US || currentRefGameState.commandId == RefCommand::INDIRECT_FREE_THEM)) {
        return refCommandToGameStateMapping.at(RefCommand::NORMAL_START);
    }

    // TODO: Why is was this needed?
    //  // if the command is the same as the previous, we don't need to do anything
    //  if (newRefCommand == currentRefCmd) {
    //    return;
    //  }

    // if the command is the same, we don't need to do anything
    if (newRefCmd == currentRefGameState.commandId) {
        return currentRefGameState;
    }

    // otherwise, if we are in a follow upstate and the refcommand is normal start we don't change a thing
    if (currentRefGameState.isfollowUpCommand && newRefCmd == RefCommand::NORMAL_START) {
        return currentRefGameState;
    }

    return refCommandToGameStateMapping.at((newRefCmd != RefCommand::NORMAL_START || !currentRefGameState.hasFollowUpCommand()) ? newRefCmd : currentRefGameState.followUpCommandId);
}

}  // namespace rtt::ai

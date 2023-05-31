#include "utilities/GameStateManager.hpp"

#include <roboteam_utils/Print.h>

#include "utilities/GameSettings.h"
#include "world/World.hpp"

namespace rtt::ai {

proto::SSL_Referee GameStateManager::refMsg;
StrategyManager GameStateManager::strategymanager;
std::mutex GameStateManager::refMsgLock;

std::string SSLRefereeStageToString(proto::SSL_Referee_Stage stage){
    switch(stage){
        case proto::SSL_Referee_Stage_NORMAL_FIRST_HALF_PRE: return "NORMAL_FIRST_HALF_PRE";    
        case proto::SSL_Referee_Stage_NORMAL_FIRST_HALF: return "NORMAL_FIRST_HALF";
        case proto::SSL_Referee_Stage_NORMAL_HALF_TIME: return "NORMAL_HALF_TIME";
        case proto::SSL_Referee_Stage_NORMAL_SECOND_HALF_PRE: return "NORMAL_SECOND_HALF_PRE";
        case proto::SSL_Referee_Stage_NORMAL_SECOND_HALF: return "NORMAL_SECOND_HALF";
        case proto::SSL_Referee_Stage_EXTRA_TIME_BREAK: return "EXTRA_TIME_BREAK";
        case proto::SSL_Referee_Stage_EXTRA_FIRST_HALF_PRE: return "EXTRA_FIRST_HALF_PRE";
        case proto::SSL_Referee_Stage_EXTRA_FIRST_HALF: return "EXTRA_FIRST_HALF";
        case proto::SSL_Referee_Stage_EXTRA_HALF_TIME: return "EXTRA_HALF_TIME";
        case proto::SSL_Referee_Stage_EXTRA_SECOND_HALF_PRE: return "EXTRA_SECOND_HALF_PRE";
        case proto::SSL_Referee_Stage_EXTRA_SECOND_HALF: return "EXTRA_SECOND_HALF";
        case proto::SSL_Referee_Stage_PENALTY_SHOOTOUT_BREAK: return "PENALTY_SHOOTOUT_BREAK";
        case proto::SSL_Referee_Stage_PENALTY_SHOOTOUT: return "PENALTY_SHOOTOUT";
        case proto::SSL_Referee_Stage_POST_GAME: return "POST_GAME";
        default: return "UNKNOWN";
    }
}

std::string SSLRefereeCommandToString(proto::SSL_Referee_Command command){
    switch(command){
        case proto::SSL_Referee_Command_HALT: return "HALT";
        case proto::SSL_Referee_Command_STOP: return "STOP";
        case proto::SSL_Referee_Command_NORMAL_START: return "NORMAL_START";
        case proto::SSL_Referee_Command_FORCE_START: return "FORCE_START";
        case proto::SSL_Referee_Command_PREPARE_KICKOFF_YELLOW: return "PREPARE_KICKOFF_YELLOW";
        case proto::SSL_Referee_Command_PREPARE_KICKOFF_BLUE: return "PREPARE_KICKOFF_BLUE";
        case proto::SSL_Referee_Command_PREPARE_PENALTY_YELLOW: return "PREPARE_PENALTY_YELLOW";
        case proto::SSL_Referee_Command_PREPARE_PENALTY_BLUE: return "PREPARE_PENALTY_BLUE";
        case proto::SSL_Referee_Command_DIRECT_FREE_YELLOW: return "DIRECT_FREE_YELLOW";
        case proto::SSL_Referee_Command_DIRECT_FREE_BLUE: return "DIRECT_FREE_BLUE";
        case proto::SSL_Referee_Command_INDIRECT_FREE_YELLOW: return "INDIRECT_FREE_YELLOW";
        case proto::SSL_Referee_Command_INDIRECT_FREE_BLUE: return "INDIRECT_FREE_BLUE";
        case proto::SSL_Referee_Command_TIMEOUT_YELLOW: return "TIMEOUT_YELLOW";
        case proto::SSL_Referee_Command_TIMEOUT_BLUE: return "TIMEOUT_BLUE";
        case proto::SSL_Referee_Command_BALL_PLACEMENT_YELLOW: return "BALL_PLACEMENT_YELLOW";
        case proto::SSL_Referee_Command_BALL_PLACEMENT_BLUE: return "BALL_PLACEMENT_BLUE";
        default: return "UNKNOWN";
    }
}

RefCommand SSLRefereeCommandToRefCommand(proto::SSL_Referee_Command command, bool isYellow) {
    switch (command){
        case proto::SSL_Referee_Command_HALT: return RefCommand::HALT;
        case proto::SSL_Referee_Command_STOP: return RefCommand::STOP;
        case proto::SSL_Referee_Command_NORMAL_START: return RefCommand::NORMAL_START;
        case proto::SSL_Referee_Command_FORCE_START: return RefCommand::FORCED_START;

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
            RTT_WARNING("Unhandled SSL_Referee_Command: ", SSLRefereeCommandToString(command));
            return RefCommand::HALT;
    } 
}

proto::SSL_Referee GameStateManager::getRefereeData() {
    std::lock_guard<std::mutex> lock(refMsgLock);
    return GameStateManager::refMsg;
}

void GameStateManager::setRefereeData(proto::SSL_Referee refMsg, const rtt::world::World* data) {
    
    if( GameStateManager::refMsg.stage() != refMsg.stage() || GameStateManager::refMsg.command() != refMsg.command() ){
        RTT_INFO("New referee state: stage =", SSLRefereeStageToString(refMsg.stage()), " command =", SSLRefereeCommandToString(refMsg.command()));
    }
    
    std::lock_guard<std::mutex> lock(refMsgLock);
    GameStateManager::refMsg = refMsg;
    RefCommand cmd = SSLRefereeCommandToRefCommand(refMsg.command(), GameSettings::isYellow());
    
    auto stage = refMsg.stage();
    auto world = data->getWorld();
    if (world.has_value()) {
        strategymanager.setCurrentRefGameState(cmd, stage, world->getBall());
    }
}

// Initialize static variables
GameState GameStateManager::getCurrentGameState() {
    GameState newGameState;
    if (interface::Output::usesRefereeCommands()) {
        newGameState = static_cast<GameState>(strategymanager.getCurrentRefGameState());

        if (GameSettings::isYellow()) {
            newGameState.keeperId = getRefereeData().yellow().goalkeeper();
        } else {
            newGameState.keeperId = getRefereeData().blue().goalkeeper();
        }
        // if there is a ref we set the interface gamestate to these values as well
        // this makes sure that when we stop using the referee we don't return to an unknown state,
        // // so now we keep the same.
        interface::Output::setInterfaceGameState(newGameState);
    } else {
        newGameState = interface::Output::getInterfaceGameState();
    }
    return newGameState;
}

void GameStateManager::forceNewGameState(RefCommand cmd, std::optional<rtt::world::view::BallView> ball) {
    RTT_INFO("Forcing new refstate!")

    // overwrite both the interface and the strategy manager.
    interface::Output::setInterfaceGameState(strategymanager.getRefGameStateForRefCommand(cmd));

    strategymanager.forceCurrentRefGameState(cmd, ball);
}

Vector2 GameStateManager::getRefereeDesignatedPosition() {
    auto designatedPos = rtt::ai::GameStateManager::getRefereeData().designated_position();
    return Vector2(designatedPos.x() / 1000, designatedPos.y() / 1000);
}

void GameStateManager::updateInterfaceGameState(const char* name) {
    if (strcmp(name, "Ball Placement Us") == 0) {
        interface::Output::setInterfaceGameState(GameState("ball_placement_us", Constants::RULESET_BALLPLACEMENT_US()));
    } else if (strcmp(name, "Halt") == 0) {
        interface::Output::setInterfaceGameState(GameState("halt", Constants::RULESET_HALT()));
    } else if (strcmp(name, "Free Kick Them") == 0) {
        interface::Output::setInterfaceGameState(GameState("free_kick_them", Constants::RULESET_STOP()));
    } else if (strcmp(name, "Free Kick Us At Goal") == 0) {
        interface::Output::setInterfaceGameState(GameState("free_kick_us", Constants::RULESET_DEFAULT()));
    } else if (strcmp(name, "Free Kick Us Pass") == 0) {
        interface::Output::setInterfaceGameState(GameState("free_kick_us", Constants::RULESET_DEFAULT()));
    } else if (strcmp(name, "Ball Placement Them") == 0) {
        interface::Output::setInterfaceGameState(GameState("ball_placement_them", Constants::RULESET_BALLPLACEMENT_THEM()));
    } else if (strcmp(name, "Kick Off Them Prepare") == 0) {
        interface::Output::setInterfaceGameState(GameState("kickoff_them_prepare", Constants::RULESET_KICKOFF()));
    } else if (strcmp(name, "Kick Off Us Prepare") == 0) {
        interface::Output::setInterfaceGameState(GameState("kickoff_us_prepare", Constants::RULESET_KICKOFF()));
    } else if (strcmp(name, "Kick Off Them") == 0) {
        interface::Output::setInterfaceGameState(GameState("kickoff_them", Constants::RULESET_DEFAULT()));
    } else if (strcmp(name, "Kick Off Us") == 0) {
        interface::Output::setInterfaceGameState(GameState("kickoff_us", Constants::RULESET_DEFAULT()));
    } else if (strcmp(name, "Penalty Them Prepare") == 0) {
        interface::Output::setInterfaceGameState(GameState("penalty_them_prepare", Constants::RULESET_DEFAULT()));
    } else if (strcmp(name, "Penalty Us Prepare") == 0) {
        interface::Output::setInterfaceGameState(GameState("penalty_us_prepare", Constants::RULESET_DEFAULT()));
    } else if (strcmp(name, "Penalty Them") == 0) {
        interface::Output::setInterfaceGameState(GameState("penalty_them", Constants::RULESET_DEFAULT()));
    } else if (strcmp(name, "Penalty Us") == 0) {
        interface::Output::setInterfaceGameState(GameState("penalty_us", Constants::RULESET_DEFAULT()));
    } else if (strcmp(name, "Time Out") == 0) {
        interface::Output::setInterfaceGameState(GameState("time_out", Constants::RULESET_DEFAULT()));
    } else if (strcmp(name, "Defensive Stop Formation") == 0) {
        interface::Output::setInterfaceGameState(GameState("stop", Constants::RULESET_STOP()));
    } else if (strcmp(name, "Aggressive Stop Formation") == 0) {
        interface::Output::setInterfaceGameState(GameState("stop", Constants::RULESET_STOP()));
    } else {
        RTT_WARNING("Play has been selected for which no ruleset is found!");
        interface::Output::setInterfaceGameState(GameState("normal_play", Constants::RULESET_DEFAULT()));
    }
}
}  // namespace rtt::ai

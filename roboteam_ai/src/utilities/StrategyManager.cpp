//
// Created by mrlukasbos on 9-11-18.
//

#include "utilities/StrategyManager.h"

#include "stp/constants/ControlConstants.h"
namespace rtt::ai {

void StrategyManager::setCurrentGameState(RefCommand command, proto::Referee_Stage stage, std::optional<world::view::BallView> ballOpt) {
    if (command == RefCommand::STOP && (stage == proto::Referee_Stage_NORMAL_FIRST_HALF_PRE || stage == proto::Referee_Stage_NORMAL_SECOND_HALF_PRE ||
                                        stage == proto::Referee_Stage_EXTRA_FIRST_HALF_PRE || stage == proto::Referee_Stage_EXTRA_SECOND_HALF_PRE)) {
        command = RefCommand::PREPARE_KICKOFF_THEM;
    }

    if ((currentGameState.commandId == RefCommand::DIRECT_FREE_THEM || currentGameState.commandId == RefCommand::DIRECT_FREE_US ||
         currentGameState.commandId == RefCommand::KICKOFF_US || currentGameState.commandId == RefCommand::KICKOFF_THEM) &&
        ballOpt.has_value() && ballOpt.value()->velocity.length() > stp::control_constants::BALL_IS_MOVING_SLOW_LIMIT) {
        currentGameState = getGameStateForRefCommand(RefCommand::NORMAL_START);
        lastCommand = command;
        return;
    }

    if (command == lastCommand || command == currentGameState.commandId || (currentGameState.isFollowUpCommand && command == RefCommand::NORMAL_START)) {
        return;
    }

    currentGameState = getGameStateForRefCommand(currentGameState.hasFollowUpCommand() && command == RefCommand::NORMAL_START ? currentGameState.followUpCommandId : command);
    lastCommand = command;
}

GameState StrategyManager::getCurrentGameState() { return currentGameState; }

const GameState StrategyManager::getGameStateForRefCommand(RefCommand command) {
    auto it = std::find_if(gameStates.begin(), gameStates.end(), [&command](const GameState &gameState) { return gameState.commandId == command; });

    if (it != gameStates.end()) {
        return *it;
    }
    std::cerr << "Returning an undefined refstate! This should never happen!" << std::endl;
    return gameStates[0];
}

void StrategyManager::forceCurrentGameState(RefCommand command) { currentGameState = getGameStateForRefCommand(command); }

}  // namespace rtt::ai
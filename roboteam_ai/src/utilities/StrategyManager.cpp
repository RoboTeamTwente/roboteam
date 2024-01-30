//
// Created by mrlukasbos on 9-11-18.
//

#include "utilities/StrategyManager.h"

#include "stp/constants/ControlConstants.h"
namespace rtt::ai {

void StrategyManager::setCurrentRefGameState(RefCommand command, proto::Referee_Stage stage, std::optional<world::view::BallView> ballOpt) {
    if (command == RefCommand::STOP && (stage == proto::Referee_Stage_NORMAL_FIRST_HALF_PRE || stage == proto::Referee_Stage_NORMAL_SECOND_HALF_PRE ||
                                        stage == proto::Referee_Stage_EXTRA_FIRST_HALF_PRE || stage == proto::Referee_Stage_EXTRA_SECOND_HALF_PRE)) {
        command = RefCommand::PREPARE_KICKOFF_THEM;
    }

    if ((currentRefGameState.commandId == RefCommand::DIRECT_FREE_THEM || currentRefGameState.commandId == RefCommand::DIRECT_FREE_US ||
         currentRefGameState.commandId == RefCommand::KICKOFF_US || currentRefGameState.commandId == RefCommand::KICKOFF_THEM) &&
        ballOpt.has_value() && ballOpt.value()->velocity.length() > stp::control_constants::BALL_IS_MOVING_SLOW_LIMIT) {
        currentRefGameState = getRefGameStateForRefCommand(RefCommand::NORMAL_START);
        lastCommand = command;
        return;
    }

    if (command == lastCommand || command == currentRefGameState.commandId || (currentRefGameState.isFollowUpCommand && command == RefCommand::NORMAL_START)) {
        return;
    }

    currentRefGameState =
        getRefGameStateForRefCommand(currentRefGameState.hasFollowUpCommand() && command == RefCommand::NORMAL_START ? currentRefGameState.followUpCommandId : command);
    lastCommand = command;
}

RefGameState StrategyManager::getCurrentRefGameState() { return currentRefGameState; }

const RefGameState StrategyManager::getRefGameStateForRefCommand(RefCommand command) {
    auto it = std::find_if(gameStates.begin(), gameStates.end(), [&command](const RefGameState &gameState) { return gameState.commandId == command; });

    if (it != gameStates.end()) {
        return *it;
    }
    std::cerr << "Returning an undefined refstate! This should never happen!" << std::endl;
    return gameStates[0];
}

void StrategyManager::forceCurrentRefGameState(RefCommand command) { currentRefGameState = getRefGameStateForRefCommand(command); }

}  // namespace rtt::ai
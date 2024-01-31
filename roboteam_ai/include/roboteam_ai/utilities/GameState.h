#ifndef ROBOTEAM_AI_GAMESTATE_H
#define ROBOTEAM_AI_GAMESTATE_H

#include <roboteam_utils/Vector2.h>

#include "Constants.h"
#include "RefCommand.h"
#include "RuleSet.h"
#include "stp/constants/ControlConstants.h"

namespace rtt::ai {

struct GameState {
    RefCommand commandId;
    RuleSet ruleSet;
    bool isFollowUpCommand;
    RefCommand followUpCommandId;
    int keeperId = Constants::DEFAULT_KEEPER_ID();
    int maxAllowedRobots = stp::control_constants::MAX_ROBOT_COUNT;
    static int cardId;

    GameState() = default;

    GameState(RefCommand commandId, RuleSet ruleSet, bool isFollowUpCommand = false, RefCommand followUpCommandId = RefCommand::UNDEFINED)
        : commandId(commandId), ruleSet(std::move(ruleSet)), isFollowUpCommand(isFollowUpCommand), followUpCommandId(followUpCommandId) {}

    bool hasFollowUpCommand() const { return followUpCommandId != RefCommand::UNDEFINED; }

    RuleSet getRuleSet() const { return ruleSet; }

    RefCommand getCommandId() const { return commandId; }
};

}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_GAMESTATE_H
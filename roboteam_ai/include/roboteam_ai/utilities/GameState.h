#ifndef ROBOTEAM_AI_GAMESTATE_H
#define ROBOTEAM_AI_GAMESTATE_H

#include <roboteam_utils/Vector2.h>

#include <optional>

#include "Constants.h"
#include "RefCommand.h"
#include "RuleSet.h"
#include "utilities/Constants.h"

namespace rtt::ai {

struct GameState {
    // Commands received from the ref
    static RefCommand commandFromRef;
    static RefCommand followUpCommandFromRef;
    // Processed commands
    RefCommand commandId;
    RuleSet ruleSet;
    bool isFollowUpCommand;
    RefCommand followUpCommandId;
    int keeperId = -1;
    int maxAllowedRobots = constants::MAX_ROBOT_COUNT;
    static int cardId;
    static double timeLeft;
    static std::optional<Vector2> kickPoint;

    GameState() = default;

    GameState(RefCommand commandId, RuleSet ruleSet, bool isFollowUpCommand = false, RefCommand followUpCommandId = RefCommand::UNDEFINED)
        : commandId(commandId), ruleSet(std::move(ruleSet)), isFollowUpCommand(isFollowUpCommand), followUpCommandId(followUpCommandId) {}

    bool hasFollowUpCommand() const { return followUpCommandId != RefCommand::UNDEFINED; }

    RuleSet getRuleSet() const { return ruleSet; }

    RefCommand getCommandId() const { return commandId; }

    RefCommand getCommandFromRef() const { return commandFromRef; }

    RefCommand getFollowUpCommandFromRef() const { return followUpCommandFromRef; }
};

}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_GAMESTATE_H
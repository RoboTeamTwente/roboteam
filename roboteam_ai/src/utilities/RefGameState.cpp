//
// Created by mrlukasbos on 11-5-19.
//

#include "utilities/RefGameState.h"

#include "utilities/RefCommand.h"

namespace rtt::ai {

RefGameState::RefGameState(RefCommand commandId, RuleSet ruleSet, bool isFollowUpCommand, RefCommand followUpCommandId)
    : GameState(commandId, std::move(ruleSet)), commandId(commandId), isfollowUpCommand(isFollowUpCommand), followUpCommandId(followUpCommandId) {}

bool RefGameState::hasFollowUpCommand() const { return followUpCommandId != RefCommand::UNDEFINED; }

}  // namespace rtt::ai
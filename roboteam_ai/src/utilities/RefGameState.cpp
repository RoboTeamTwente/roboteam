//
// Created by mrlukasbos on 11-5-19.
//

#include "utilities/RefGameState.h"
#include "utilities/RefCommand.h"

namespace rtt::ai {

RefGameState::RefGameState(RefCommand commandId, std::string strategyName, RuleSet ruleSet, bool isFollowUpCommand, RefCommand followUpCommandId)
    : GameState(std::move(strategyName), std::move(ruleSet)), commandId(commandId), isfollowUpCommand(isFollowUpCommand), followUpCommandId(followUpCommandId) {}

bool RefGameState::hasFollowUpCommand() const { return followUpCommandId != RefCommand::UNDEFINED; }

}  // namespace rtt::ai

inline std::ostream& operator<<(std::ostream& os, const rtt::ai::RefGameState& gs) {
    os << "RefGameState{"
       << "\n.strategyName = " << gs.getStrategyName()
       << "\n.ruleSetName = " << gs.ruleSetName
       << "\n.keeperId = " << gs.keeperId
       << "\n.commandId = " << static_cast<std::underlying_type<rtt::RefCommand>::type>(gs.commandId)
       << "\n.isFollowUpCommand = " << gs.isfollowUpCommand
       << "\n.followUpCommandId = " << static_cast<std::underlying_type<rtt::RefCommand>::type>(gs.followUpCommandId)
    << "\n}";
    return os;
}// namespace rtt::ai
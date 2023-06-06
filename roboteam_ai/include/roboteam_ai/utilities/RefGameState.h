//
// Created by mrlukasbos on 11-5-19.
//

#ifndef ROBOTEAM_AI_REFGAMESTATE_H
#define ROBOTEAM_AI_REFGAMESTATE_H

#include <roboteam_utils/Vector2.h>

#include "Constants.h"
#include "GameState.h"
#include "RuleSet.h"

namespace rtt::ai {

/**
 * @brief Structure that defines the game state from the referee. This is a child structure of the GameState structure
 */
struct RefGameState : public GameState {
    RefCommand commandId; /**< The command that has been sent by the referee */
    bool isfollowUpCommand; /**< Indicates whether the command is a follow up command from the previous command */
    RefCommand followUpCommandId; /**< The follow up command that has been sent by the referee */

    /**
     * @brief Default constructor for the RefGameState structure
     */
    RefGameState() = default;

    /**
     * @brief Constructor for the RefGameState structure
     * @param commandId The command that has been send by the referee
     * @param strategyName The name of the play that belongs to this command
     * @param ruleSet The ruleset that belongs to the command
     * @param isFollowUpCommand Indicates whether the command is a follow up command from the previous command
     * @param followUpCommandId The follow up command that has been sent by the referee
     */
    RefGameState(RefCommand commandId, std::string strategyName, RuleSet ruleSet, bool isFollowUpCommand = false, RefCommand followUpCommandId = RefCommand::UNDEFINED);

    /**
     * @brief Checks whether this command has a follow up command
     * @return Boolean that tells whether this command has a follow up command
     */
    bool hasFollowUpCommand() const;
};

}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_REFGAMESTATE_H

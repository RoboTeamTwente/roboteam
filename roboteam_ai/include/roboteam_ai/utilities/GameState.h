#ifndef ROBOTEAM_AI_GAMESTATE_H
#define ROBOTEAM_AI_GAMESTATE_H

#include <roboteam_utils/Print.h>
#include <roboteam_utils/Vector2.h>

#include "Constants.h"
#include "RuleSet.h"
#include "stp/constants/ControlConstants.h"
#include "utilities/RefCommand.h"

namespace rtt::ai {
/**
 * @brief Structure that contains the GameState and its respective properties
 */
struct GameState {
    /**
     * @brief Default constructor for the GameState class
     */
    GameState() = default;

    /**
     * @brief Constructor for the GameState class
     * @param strategyName Name of the game state
     * @param ruleSet Ruleset that belongs to the game state
     */
    GameState(RefCommand strategyName, RuleSet ruleSet) : ruleSet(std::move(ruleSet)), strategyName(std::move(strategyName)){};

    RuleSet ruleSet;
    int keeperId = Constants::DEFAULT_KEEPER_ID();
    int maxAllowedRobots = stp::control_constants::MAX_ROBOT_COUNT;
    static int cardId;

    /**
     * @brief Getter for the ruleset according to its name
     * @return Ruleset that belongs to the name
     */
    RuleSet getRuleSet() const { return ruleSet; }

    /**
     * @brief Getter for the name of the current game state
     * @return The name of the current game state
     */
    RefCommand getStrategyName() const { return strategyName; }

   private:
    RefCommand strategyName; /**< The name of the current game state */
};
}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_GAMESTATE_H

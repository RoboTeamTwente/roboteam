#ifndef ROBOTEAM_AI_GAMESTATE_H
#define ROBOTEAM_AI_GAMESTATE_H

#include <roboteam_utils/Vector2.h>

#include "Constants.h"
#include "RuleSet.h"

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
     * @param ruleSetName Name of the ruleset that belongs to the game state
     */
    GameState(std::string strategyName, std::string ruleSetName) : ruleSetName(std::move(ruleSetName)), strategyName(std::move(strategyName)){};

    std::string ruleSetName; /**< Name of the ruleset that belongs to the game state */
    int keeperId = Constants::DEFAULT_KEEPER_ID(); /**< ID of the keeper */

    /**
     * @brief Getter for the ruleset according to its name
     * @return Ruleset that belongs to the name
     */
    RuleSet getRuleSet() {
        for (auto ruleSet : Constants::ruleSets()) {
            if (ruleSet.title == ruleSetName) {
                return ruleSet;
            }
        }
        std::cerr << "Returning empty ruleset with name '" << ruleSetName << "', this should never happen!" << std::endl;
        return {};
    }

    /**
     * @brief Getter for the name of the current game state
     * @return The name of the current game state
     */
    std::string getStrategyName() { return strategyName; }

   private:
    std::string strategyName; /**< The name of the current game state */
};

}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_GAMESTATE_H

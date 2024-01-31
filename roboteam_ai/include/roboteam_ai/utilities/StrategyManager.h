/*
 * Created by mrlukasbos on 9-11-18.
 */

#ifndef ROBOTEAM_AI_STRATEGYMANAGER_H
#define ROBOTEAM_AI_STRATEGYMANAGER_H

#include <proto/messages_robocup_ssl_referee.pb.h>

#include <iostream>
#include <map>

#include "Constants.h"
#include "GameState.h"
#include "world/views/WorldDataView.hpp"

namespace rtt::ai {

/**
 * @brief Class that defines the StrategyManager. The StrategyManager manages the rule sets of each game state
 */
class StrategyManager {
   public:
    /**
     * @brief Default constructor for the StrategyManager class
     */
    explicit StrategyManager() = default;

    /**
     * @brief Gets the current game state from the referee
     * @return the current game state from the referee
     */
    GameState getCurrentGameState();

    /**
     * @brief Sets the current game state from the referee
     * @param command Game state the referee is giving
     * @param nextCommand Game state the referee is giving after the current one
     * @param ballOpt Data about the ball
     */
    void setCurrentGameState(RefCommand command, RefCommand nextCommand, std::optional<rtt::world::view::BallView> ballOpt);

    /**
     * @brief Forces the AI into a given game state
     * @param command The game state that should be considered
     */
    void forceCurrentGameState(RefCommand command);

    /**
     * @brief Gets the game state that belongs to the given command
     * @param command Command given by the referee
     * @return Game state that belongs to the given command
     */
    const GameState getGameStateForRefCommand(RefCommand command);

   private:
    /**
     * @brief Vector containing all possible game states
     */
    const std::vector<GameState> gameStates = {
        GameState(RefCommand::UNDEFINED, Constants::RULESET_HALT()),
        GameState(RefCommand::HALT, Constants::RULESET_HALT()),
        GameState(RefCommand::TIMEOUT_US, Constants::RULESET_HALT()),
        GameState(RefCommand::TIMEOUT_THEM, Constants::RULESET_HALT()),

        GameState(RefCommand::STOP, Constants::RULESET_STOP()),
        GameState(RefCommand::BALL_PLACEMENT_THEM, Constants::RULESET_STOP()),
        GameState(RefCommand::BALL_PLACEMENT_US, Constants::RULESET_STOP()),
        GameState(RefCommand::BALL_PLACEMENT_US_DIRECT, Constants::RULESET_STOP()),
        GameState(RefCommand::PREPARE_KICKOFF_US, Constants::RULESET_STOP(), false, RefCommand::KICKOFF_US),
        GameState(RefCommand::PREPARE_KICKOFF_THEM, Constants::RULESET_STOP(), false, RefCommand::KICKOFF_THEM),
        GameState(RefCommand::PREPARE_PENALTY_US, Constants::RULESET_STOP(), false, RefCommand::PENALTY_US),
        GameState(RefCommand::PREPARE_PENALTY_THEM, Constants::RULESET_STOP(), false, RefCommand::PENALTY_THEM),

        GameState(RefCommand::DIRECT_FREE_THEM, Constants::RULESET_DEFAULT()),
        GameState(RefCommand::NORMAL_START, Constants::RULESET_DEFAULT()),
        GameState(RefCommand::FORCED_START, Constants::RULESET_DEFAULT()),
        GameState(RefCommand::DIRECT_FREE_US, Constants::RULESET_DEFAULT()),
        GameState(RefCommand::KICKOFF_US, Constants::RULESET_DEFAULT(), true),
        GameState(RefCommand::KICKOFF_THEM, Constants::RULESET_DEFAULT(), true),
        GameState(RefCommand::PENALTY_US, Constants::RULESET_DEFAULT(), true),
        GameState(RefCommand::PENALTY_THEM, Constants::RULESET_DEFAULT(), true),
    };
    GameState currentGameState = gameStates[0];     /**< Current game state according to the referee, after the StrategyManager has processed it */
    RefCommand lastCommand = RefCommand::UNDEFINED; /**< Last command given by the referee */
};

}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_STRATEGYMANAGER_H

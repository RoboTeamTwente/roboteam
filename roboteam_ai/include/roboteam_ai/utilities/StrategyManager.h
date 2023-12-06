/*
 * Created by mrlukasbos on 9-11-18.
 */

#ifndef ROBOTEAM_AI_STRATEGYMANAGER_H
#define ROBOTEAM_AI_STRATEGYMANAGER_H

#include <proto/messages_robocup_ssl_referee.pb.h>

#include <iostream>
#include <map>

#include "Constants.h"
#include "RefGameState.h"
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
    RefGameState getCurrentRefGameState();

    /**
     * @brief Sets the current game state from the referee
     * @param command Game state the referee is giving
     * @param stage Stage of the game
     * @param ballOpt Data about the ball
     */
    void setCurrentRefGameState(RefCommand command, proto::SSL_Referee_Stage stage, std::optional<rtt::world::view::BallView> ballOpt);

    /**
     * @brief Forces the AI into a given game state
     * @param command The game state that should be considered
     */
    void forceCurrentRefGameState(RefCommand command);

    /**
     * @brief Gets the game state that belongs to the given command
     * @param command Command given by the referee
     * @return Game state that belongs to the given command
     */
    const RefGameState getRefGameStateForRefCommand(RefCommand command);

   private:
    /**
     * @brief Vector containing all possible game states
     */
    const std::vector<RefGameState> gameStates = {
        RefGameState(RefCommand::UNDEFINED, "halt", Constants::RULESET_HALT()),
        RefGameState(RefCommand::HALT, "halt", Constants::RULESET_HALT()),
        RefGameState(RefCommand::TIMEOUT_US, "halt", Constants::RULESET_HALT()),
        RefGameState(RefCommand::TIMEOUT_THEM, "halt", Constants::RULESET_HALT()),

        RefGameState(RefCommand::STOP, "stop", Constants::RULESET_STOP()),
        RefGameState(RefCommand::BALL_PLACEMENT_THEM, "ball_placement_them", Constants::RULESET_BALLPLACEMENT_THEM()),
        RefGameState(RefCommand::BALL_PLACEMENT_US, "ball_placement_us", Constants::RULESET_BALLPLACEMENT_US()),

        RefGameState(RefCommand::NORMAL_START, "normal_play", Constants::RULESET_DEFAULT()),
        RefGameState(RefCommand::FORCED_START, "normal_play", Constants::RULESET_DEFAULT()),

        RefGameState(RefCommand::DIRECT_FREE_THEM, "free_kick_them", Constants::RULESET_STOP()),
        RefGameState(RefCommand::INDIRECT_FREE_THEM, "free_kick_them", Constants::RULESET_STOP()),

        RefGameState(RefCommand::PREPARE_KICKOFF_US, "kickoff_us_prepare", Constants::RULESET_KICKOFF(), false, RefCommand::DO_KICKOFF),
        RefGameState(RefCommand::PREPARE_KICKOFF_THEM, "kickoff_them_prepare", Constants::RULESET_KICKOFF(), false, RefCommand::DEFEND_KICKOFF),

        RefGameState(RefCommand::DIRECT_FREE_US, "free_kick_us", Constants::RULESET_DEFAULT()),
        RefGameState(RefCommand::INDIRECT_FREE_US, "free_kick_us", Constants::RULESET_DEFAULT()),
        RefGameState(RefCommand::GOAL_US, "kickoff_them_prepare", Constants::RULESET_DEFAULT()),
        RefGameState(RefCommand::GOAL_THEM, "kickoff_us_prepare", Constants::RULESET_DEFAULT()),
        RefGameState(RefCommand::PREPARE_PENALTY_US, "penalty_us_prepare", Constants::RULESET_DEFAULT(), false, RefCommand::DO_PENALTY),
        RefGameState(RefCommand::PREPARE_PENALTY_THEM, "penalty_them_prepare", Constants::RULESET_DEFAULT(), false, RefCommand::DEFEND_PENALTY),
        RefGameState(RefCommand::DO_KICKOFF, "kickoff_us", Constants::RULESET_DEFAULT(), true),
        RefGameState(RefCommand::DEFEND_KICKOFF, "kickoff_them", Constants::RULESET_DEFAULT(), true),
        RefGameState(RefCommand::DO_PENALTY, "penalty_us", Constants::RULESET_DEFAULT(), true),
        RefGameState(RefCommand::DEFEND_PENALTY, "penalty_them", Constants::RULESET_DEFAULT(), true),
        RefGameState(RefCommand::DO_SHOOTOUT, "time_out", Constants::RULESET_DEFAULT(), true),
        RefGameState(RefCommand::DEFEND_SHOOTOUT, "time_out", Constants::RULESET_DEFAULT(), true),
        RefGameState(RefCommand::PREPARE_SHOOTOUT_US, "penalty_us_prepare", Constants::RULESET_DEFAULT(), false, RefCommand::DO_PENALTY),
        RefGameState(RefCommand::PREPARE_SHOOTOUT_THEM, "penalty_them_prepare", Constants::RULESET_DEFAULT(), false, RefCommand::DEFEND_SHOOTOUT),
        RefGameState(RefCommand::PRE_HALF, "formation_pre_half", Constants::RULESET_STOP(), false)};

    RefGameState currentRefGameState = gameStates[0]; /**< Current game state according to the referee */
    RefCommand currentRefCmd = RefCommand::UNDEFINED; /**< Current command given by the referee */
};

}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_STRATEGYMANAGER_H

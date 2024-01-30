//
// Created by Alexander on 28-01-2022
//

#include "stp/evaluations/game_states/KickOffUsOrNormalGameStateEvaluation.h"

#include "utilities/GameStateManager.hpp"

namespace rtt::ai::stp::evaluation {

uint8_t KickOffUsOrNormalGameStateEvaluation::metricCheck(const world::World *, const Field *) const noexcept {
    return (GameStateManager::getCurrentGameState().getStrategyName() == RefCommand::KICKOFF_US ||
            GameStateManager::getCurrentGameState().getStrategyName() == RefCommand::NORMAL_START)
               ? stp::control_constants::FUZZY_TRUE
               : stp::control_constants::FUZZY_FALSE;
}
}  // namespace rtt::ai::stp::evaluation
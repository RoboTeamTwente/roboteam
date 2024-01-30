//
// Created by jordi on 28-04-20.
//

#include "stp/evaluations/game_states/PenaltyThemGameStateEvaluation.h"

#include "utilities/GameStateManager.hpp"

namespace rtt::ai::stp::evaluation {

uint8_t PenaltyThemGameStateEvaluation::metricCheck(const world::World *, const Field *) const noexcept {
    return GameStateManager::getCurrentGameState().getStrategyName() == RefCommand::PENALTY_THEM ? stp::control_constants::FUZZY_TRUE : stp::control_constants::FUZZY_FALSE;
}
}  // namespace rtt::ai::stp::evaluation
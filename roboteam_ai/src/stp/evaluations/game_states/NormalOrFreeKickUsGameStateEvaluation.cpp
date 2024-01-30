//
// Created by ratoone on 15-05-20.
//

#include "stp/evaluations/game_states/NormalOrFreeKickUsGameStateEvaluation.h"

#include "utilities/GameStateManager.hpp"

namespace rtt::ai::stp::evaluation {

uint8_t NormalOrFreeKickUsGameStateEvaluation::metricCheck(const world::World *, const Field *) const noexcept {
    return (GameStateManager::getCurrentGameState().getStrategyName() == RefCommand::DIRECT_FREE_US ||
            GameStateManager::getCurrentGameState().getStrategyName() == RefCommand::NORMAL_START)
               ? stp::control_constants::FUZZY_TRUE
               : stp::control_constants::FUZZY_FALSE;
}
}  // namespace rtt::ai::stp::evaluation
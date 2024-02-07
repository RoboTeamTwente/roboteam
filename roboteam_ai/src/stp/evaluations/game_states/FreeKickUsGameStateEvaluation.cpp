//
// Created by jordi on 28-04-20.
//

#include "stp/evaluations/game_states/FreeKickUsGameStateEvaluation.h"

#include "utilities/GameStateManager.hpp"

namespace rtt::ai::stp::evaluation {

uint8_t FreeKickUsGameStateEvaluation::metricCheck(const world::World *, const Field *) const noexcept {
    return (GameStateManager::getCurrentGameState().getCommandId() == RefCommand::DIRECT_FREE_US ||
            GameStateManager::getCurrentGameState().getCommandId() == RefCommand::DIRECT_FREE_US_STOP)
               ? stp::control_constants::FUZZY_TRUE
               : stp::control_constants::FUZZY_FALSE;
}
}  // namespace rtt::ai::stp::evaluation
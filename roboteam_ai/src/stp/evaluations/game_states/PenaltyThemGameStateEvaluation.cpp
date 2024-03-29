#include "stp/evaluations/game_states/PenaltyThemGameStateEvaluation.h"

#include "utilities/GameStateManager.hpp"

namespace rtt::ai::stp::evaluation {

uint8_t PenaltyThemGameStateEvaluation::metricCheck(const world::World *, const Field *) const noexcept {
    return GameStateManager::getCurrentGameState().getCommandId() == RefCommand::PENALTY_THEM ? stp::control_constants::FUZZY_TRUE : stp::control_constants::FUZZY_FALSE;
}
}  // namespace rtt::ai::stp::evaluation
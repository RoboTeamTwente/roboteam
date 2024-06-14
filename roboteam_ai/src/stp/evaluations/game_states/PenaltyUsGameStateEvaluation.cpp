#include "stp/evaluations/game_states/PenaltyUsGameStateEvaluation.h"

#include "utilities/GameStateManager.hpp"

namespace rtt::ai::stp::evaluation {

uint8_t PenaltyUsGameStateEvaluation::metricCheck(const world::World *, const Field *) const noexcept {
    return GameStateManager::getCurrentGameState().getCommandId() == RefCommand::PENALTY_US ? constants::FUZZY_TRUE : constants::FUZZY_FALSE;
}
}  // namespace rtt::ai::stp::evaluation
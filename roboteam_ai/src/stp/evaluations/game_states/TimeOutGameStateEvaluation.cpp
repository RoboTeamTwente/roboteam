#include "stp/evaluations/game_states/TimeOutGameStateEvaluation.h"

#include "utilities/GameStateManager.hpp"

namespace rtt::ai::stp::evaluation {
uint8_t TimeOutGameStateEvaluation::metricCheck(const world::World *, const Field *) const noexcept {
    return GameStateManager::getCurrentGameState().getCommandId() == RefCommand::TIMEOUT_THEM || GameStateManager::getCurrentGameState().getCommandId() == RefCommand::TIMEOUT_US
               ? constants::FUZZY_TRUE
               : constants::FUZZY_FALSE;
}
}  // namespace rtt::ai::stp::evaluation
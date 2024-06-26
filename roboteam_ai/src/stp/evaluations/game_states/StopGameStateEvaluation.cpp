#include "stp/evaluations/game_states/StopGameStateEvaluation.h"

#include "utilities/GameStateManager.hpp"

namespace rtt::ai::stp::evaluation {

uint8_t StopGameStateEvaluation::metricCheck(const world::World*, const Field*) const noexcept {
    return GameStateManager::getCurrentGameState().getCommandId() == RefCommand::STOP ? constants::FUZZY_TRUE : constants::FUZZY_FALSE;
}
}  // namespace rtt::ai::stp::evaluation
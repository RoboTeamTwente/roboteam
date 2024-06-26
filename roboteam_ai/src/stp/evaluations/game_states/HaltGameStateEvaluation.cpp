#include "stp/evaluations/game_states/HaltGameStateEvaluation.h"

#include "utilities/GameStateManager.hpp"

namespace rtt::ai::stp::evaluation {
uint8_t HaltGameStateEvaluation::metricCheck(const world::World*, const Field*) const noexcept {
    RefCommand strategyName = GameStateManager::getCurrentGameState().getCommandId();
    return (strategyName == RefCommand::HALT || strategyName == RefCommand::TIMEOUT_US || strategyName == RefCommand::TIMEOUT_THEM || strategyName == RefCommand::UNDEFINED)
               ? constants::FUZZY_TRUE
               : constants::FUZZY_FALSE;
}
}  // namespace rtt::ai::stp::evaluation

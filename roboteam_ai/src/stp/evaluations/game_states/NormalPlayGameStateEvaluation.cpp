#include "stp/evaluations/game_states/NormalPlayGameStateEvaluation.h"

#include "utilities/GameStateManager.hpp"

namespace rtt::ai::stp::evaluation {

uint8_t NormalPlayGameStateEvaluation::metricCheck(const world::World*, const Field*) const noexcept {
    return GameStateManager::getCurrentGameState().getCommandId() == RefCommand::NORMAL_START ? constants::FUZZY_TRUE : constants::FUZZY_FALSE;
}
}  // namespace rtt::ai::stp::evaluation
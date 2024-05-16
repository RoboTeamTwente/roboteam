#include "stp/evaluations/game_states/FreeKickThemGameStateEvaluation.h"

#include "utilities/GameStateManager.hpp"

namespace rtt::ai::stp::evaluation {

uint8_t FreeKickThemGameStateEvaluation::metricCheck(const world::World *, const Field *) const noexcept {
    return (GameStateManager::getCurrentGameState().getCommandId() == RefCommand::DIRECT_FREE_THEM ||
            GameStateManager::getCurrentGameState().getCommandId() == RefCommand::DIRECT_FREE_THEM_STOP)
               ? stp::control_constants::FUZZY_TRUE
               : stp::control_constants::FUZZY_FALSE;
}
}  // namespace rtt::ai::stp::evaluation
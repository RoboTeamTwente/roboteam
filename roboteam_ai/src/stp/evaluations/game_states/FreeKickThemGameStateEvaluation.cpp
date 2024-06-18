#include "stp/evaluations/game_states/FreeKickThemGameStateEvaluation.h"

#include "utilities/GameStateManager.hpp"

namespace rtt::ai::stp::evaluation {

uint8_t FreeKickThemGameStateEvaluation::metricCheck(const world::World *, const Field *) const noexcept {
    return (GameStateManager::getCurrentGameState().getCommandId() == RefCommand::DIRECT_FREE_THEM ||
            GameStateManager::getCurrentGameState().getCommandId() == RefCommand::DIRECT_FREE_THEM_STOP)
               ? constants::FUZZY_TRUE
               : constants::FUZZY_FALSE;
}
}  // namespace rtt::ai::stp::evaluation
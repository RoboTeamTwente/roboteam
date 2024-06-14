#include "stp/evaluations/game_states/KickOffThemPrepareGameStateEvaluation.h"

#include "utilities/GameStateManager.hpp"

namespace rtt::ai::stp::evaluation {

uint8_t KickOffThemPrepareGameStateEvaluation::metricCheck(const world::World *, const Field *) const noexcept {
    return GameStateManager::getCurrentGameState().getCommandId() == RefCommand::PREPARE_KICKOFF_THEM ? constants::FUZZY_TRUE : constants::FUZZY_FALSE;
}
}  // namespace rtt::ai::stp::evaluation
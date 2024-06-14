#include "stp/evaluations/game_states/BallPlacementUsDirectGameStateEvaluation.h"

#include "utilities/GameStateManager.hpp"

namespace rtt::ai::stp::evaluation {
uint8_t BallPlacementUsDirectGameStateEvaluation::metricCheck(const world::World *, const Field *) const noexcept {
    return (GameStateManager::getCurrentGameState().getCommandId() == RefCommand::BALL_PLACEMENT_US_DIRECT) ? constants::FUZZY_TRUE : constants::FUZZY_FALSE;
}
}  // namespace rtt::ai::stp::evaluation
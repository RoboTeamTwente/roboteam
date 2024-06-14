#include "stp/PlayEvaluator.h"

#include <stp/evaluations/game_states/BallPlacementThemGameStateEvaluation.h>
#include <stp/evaluations/game_states/BallPlacementUsDirectGameStateEvaluation.h>
#include <stp/evaluations/game_states/BallPlacementUsGameStateEvaluation.h>
#include <stp/evaluations/game_states/FreeKickThemGameStateEvaluation.h>
#include <stp/evaluations/game_states/FreeKickUsGameStateEvaluation.h>
#include <stp/evaluations/game_states/HaltGameStateEvaluation.h>
#include <stp/evaluations/game_states/KickOffThemGameStateEvaluation.h>
#include <stp/evaluations/game_states/KickOffThemPrepareGameStateEvaluation.h>
#include <stp/evaluations/game_states/KickOffUsGameStateEvaluation.h>
#include <stp/evaluations/game_states/KickOffUsPrepareGameStateEvaluation.h>
#include <stp/evaluations/game_states/NormalPlayGameStateEvaluation.h>
#include <stp/evaluations/game_states/PenaltyThemGameStateEvaluation.h>
#include <stp/evaluations/game_states/PenaltyThemPrepareGameStateEvaluation.h>
#include <stp/evaluations/game_states/PenaltyUsGameStateEvaluation.h>
#include <stp/evaluations/game_states/PenaltyUsPrepareGameStateEvaluation.h>
#include <stp/evaluations/game_states/PrepareForcedStartGameStateEvaluation.h>
#include <stp/evaluations/game_states/StopGameStateEvaluation.h>
#include <stp/evaluations/game_states/TimeOutGameStateEvaluation.h>
#include <stp/evaluations/global/BallInOurDefenseAreaAndStillGlobalEvaluation.h>
#include <stp/evaluations/global/BallOnOurSideGlobalEvaluation.h>
#include <stp/evaluations/global/TheyHaveBallGlobalEvaluation.h>
#include <stp/evaluations/global/WeWillHaveBallGlobalEvaluation.h>

namespace rtt::ai::stp {

uint8_t PlayEvaluator::getGlobalEvaluation(GlobalEvaluation evaluation, const rtt::world::World* world) {
    return (scoresGlobal.contains(evaluation) ? scoresGlobal.at(evaluation) : scoresGlobal[evaluation] = updateGlobalEvaluation(evaluation, world));
}

uint8_t PlayEvaluator::updateGlobalEvaluation(GlobalEvaluation& evaluation, const rtt::world::World* world) {
    auto field = world->getField().value();
    switch (evaluation) {
        case GlobalEvaluation::PrepareForcedStartGameState:
            return evaluation::PrepareForcedStartGameStateEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::BallPlacementThemGameState:
            return evaluation::BallPlacementThemGameStateEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::BallPlacementUsGameState:
            return evaluation::BallPlacementUsGameStateEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::BallPlacementUsDirectGameState:
            return evaluation::BallPlacementUsDirectGameStateEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::FreeKickThemGameState:
            return evaluation::FreeKickThemGameStateEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::FreeKickUsGameState:
            return evaluation::FreeKickUsGameStateEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::HaltGameState:
            return evaluation::HaltGameStateEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::TimeOutGameState:
            return evaluation::TimeOutGameStateEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::KickOffThemGameState:
            return evaluation::KickOffThemGameStateEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::KickOffThemPrepareGameState:
            return evaluation::KickOffThemPrepareGameStateEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::KickOffUsGameState:
            return evaluation::KickOffUsGameStateEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::KickOffUsPrepareGameState:
            return evaluation::KickOffUsPrepareGameStateEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::NormalPlayGameState:
            return evaluation::NormalPlayGameStateEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::PenaltyThemGameState:
            return evaluation::PenaltyThemGameStateEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::PenaltyThemPrepareGameState:
            return evaluation::PenaltyThemPrepareGameStateEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::PenaltyUsGameState:
            return evaluation::PenaltyUsGameStateEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::PenaltyUsPrepareGameState:
            return evaluation::PenaltyUsPrepareGameStateEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::StopGameState:
            return evaluation::StopGameStateEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::BallOnOurSide:
            return evaluation::BallOnOurSideGlobalEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::BallOnTheirSide:
            return constants::FUZZY_TRUE - evaluation::BallOnOurSideGlobalEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::BallInOurDefenseAreaAndStill:
            return evaluation::BallInOurDefenseAreaAndStillGlobalEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::BallNotInOurDefenseAreaAndStill:
            return constants::FUZZY_TRUE - evaluation::BallInOurDefenseAreaAndStillGlobalEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::WeWillHaveBall:
            return evaluation::WeWillHaveBallGlobalEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::WeWillNotHaveBall:
            return constants::FUZZY_TRUE - evaluation::WeWillHaveBallGlobalEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::TheyHaveBall:
            return evaluation::TheyHaveBallGlobalEvaluation().metricCheck(world, &field);
        default:
            RTT_WARNING("Unhandled ScoreEvaluation!");
            return 0;
    }
}
void PlayEvaluator::clearGlobalScores() { scoresGlobal.clear(); }

bool PlayEvaluator::checkEvaluation(GlobalEvaluation globalEvaluation, const rtt::world::World* world, uint8_t cutOff) noexcept {
    return getGlobalEvaluation(globalEvaluation, world) >= cutOff;
}

}  // namespace rtt::ai::stp
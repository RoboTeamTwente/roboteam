#include "stp/plays/referee_specific/PenaltyThemPrepare.h"

#include "stp/roles/passive/BallAvoider.h"
#include "stp/roles/passive/Formation.h"

namespace rtt::ai::stp::play {

PenaltyThemPrepare::PenaltyThemPrepare() : Play() {
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(eval::PenaltyThemPrepareGameState);

    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(eval::PenaltyThemPrepareGameState);

    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
        // Roles is we play 6v6
        std::make_unique<role::Formation>("keeper"),
        std::make_unique<role::BallAvoider>("formation_0"),
        std::make_unique<role::BallAvoider>("formation_1"),
        std::make_unique<role::BallAvoider>("formation_2"),
        std::make_unique<role::BallAvoider>("formation_3"),
        std::make_unique<role::BallAvoider>("formation_4"),
        // Additional roles if we play 11v11
        std::make_unique<role::BallAvoider>("formation_5"),
        std::make_unique<role::BallAvoider>("formation_6"),
        std::make_unique<role::BallAvoider>("formation_7"),
        std::make_unique<role::BallAvoider>("formation_8"),
        std::make_unique<role::BallAvoider>("formation_9"),
    };
}

uint8_t PenaltyThemPrepare::score(const rtt::Field& field) noexcept {
    /// List of all factors that combined results in an evaluation how good the play is.
    scoring = {{PlayEvaluator::getGlobalEvaluation(eval::PenaltyThemPrepareGameState, world), 1.0}};
    return (lastScore = PlayEvaluator::calculateScore(scoring)).value();  // DONT TOUCH.
}

Dealer::FlagMap PenaltyThemPrepare::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {}}});
    flagMap.insert({"formation_0", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_1", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_2", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_3", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_4", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_5", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_6", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_7", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_8", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_9", {DealerFlagPriority::LOW_PRIORITY, {}}});

    return flagMap;
}

void PenaltyThemPrepare::calculateInfoForRoles() noexcept {
    PositionComputations::calculateInfoForKeeper(stpInfos, field, world);
    PositionComputations::calculateInfoForPenalty(stpInfos, field, world);
}

const char* PenaltyThemPrepare::getName() const { return "Penalty Them Prepare"; }

}  // namespace rtt::ai::stp::play

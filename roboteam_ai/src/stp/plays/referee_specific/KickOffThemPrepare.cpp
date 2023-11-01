//
// Created by jordi on 30-04-20.
//

#include "stp/plays/referee_specific/KickOffThemPrepare.h"

#include "stp/roles/passive/BallAvoider.h"
#include "stp/roles/passive/Formation.h"

namespace rtt::ai::stp::play {

KickOffThemPrepare::KickOffThemPrepare() : Play() {
    /// Evaluations that have to be true to be considered when changing plays.
    startPlayEvaluation.clear();  // DONT TOUCH.
    startPlayEvaluation.emplace_back(eval::KickOffThemPrepareGameState);

    /// Evaluations that have to be true to allow the play to continue, otherwise the play will change. Plays can also end using the shouldEndPlay().
    keepPlayEvaluation.clear();  // DONT TOUCH.
    keepPlayEvaluation.emplace_back(eval::KickOffThemPrepareGameState);

    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
        // Roles is we play 6v6
        std::make_unique<role::Formation>("keeper"),
        std::make_unique<role::BallAvoider>("formation_back_0"),
        std::make_unique<role::BallAvoider>("formation_mid_0"),
        std::make_unique<role::BallAvoider>("formation_front_0"),
        std::make_unique<role::BallAvoider>("formation_back_1"),
        std::make_unique<role::BallAvoider>("formation_mid_1"),
        // Additional roles if we play 11v11
        std::make_unique<role::BallAvoider>("formation_front_1"),
        std::make_unique<role::BallAvoider>("formation_back_2"),
        std::make_unique<role::BallAvoider>("formation_mid_2"),
        std::make_unique<role::BallAvoider>("formation_front_2"),
        std::make_unique<role::BallAvoider>("formation_mid_3"),
    };
}

uint8_t KickOffThemPrepare::score(const rtt::Field& field) noexcept {
    /// List of all factors that combined results in an evaluation how good the play is.
    scoring = {{PlayEvaluator::getGlobalEvaluation(eval::KickOffThemPrepareGameState, world), 1.0}};
    return (lastScore = PlayEvaluator::calculateScore(scoring)).value();
}

Dealer::FlagMap KickOffThemPrepare::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {}}});
    flagMap.insert({"formation_back_0", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"formation_back_1", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"formation_back_2", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"formation_mid_0", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_mid_1", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_mid_2", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_mid_3", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_front_0", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"formation_front_1", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"formation_front_2", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});

    return flagMap;
}

void KickOffThemPrepare::calculateInfoForRoles() noexcept {
    PositionComputations::calculateInfoForKeeper(stpInfos, field, world);
    PositionComputations::calculateInfoForFormationOurSide(stpInfos, roles, field, world);
}

const char* KickOffThemPrepare::getName() const { return "Kick Off Them Prepare"; }

}  // namespace rtt::ai::stp::play
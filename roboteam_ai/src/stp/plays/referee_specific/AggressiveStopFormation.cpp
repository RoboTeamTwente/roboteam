//
// Created by timovdk on 3/30/20.
//

#include "stp/plays/referee_specific/AggressiveStopFormation.h"

#include "stp/roles/passive/BallAvoider.h"

namespace rtt::ai::stp::play {
AggressiveStopFormation::AggressiveStopFormation() : Play() {
    /// Evaluations that have to be true to be considered when changing plays.
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(eval::StopGameState);
    startPlayEvaluation.emplace_back(eval::BallOnTheirSide);

    /// Evaluations that have to be true to allow the play to continue, otherwise the play will change. Plays can also end using the shouldEndPlay().
    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(eval::StopGameState);

    /// Role creation, the names should be unique. The names are used in the stpInfos-map.
    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
        // Roles is we play 6v6
        std::make_unique<role::BallAvoider>("keeper"),
        std::make_unique<role::BallAvoider>("formation_back_0"),
        std::make_unique<role::BallAvoider>("formation_mid_0"),
        std::make_unique<role::BallAvoider>("formation_front_0"),
        std::make_unique<role::BallAvoider>("formation_front_1"),
        std::make_unique<role::BallAvoider>("formation_mid_1"),
        // Additional roles if we play 11v11
        std::make_unique<role::BallAvoider>("formation_back_1"),
        std::make_unique<role::BallAvoider>("formation_front_2"),
        std::make_unique<role::BallAvoider>("formation_mid_2"),
        std::make_unique<role::BallAvoider>("formation_back_2"),
        std::make_unique<role::BallAvoider>("formation_front_3"),
    };
}

uint8_t AggressiveStopFormation::score(const rtt::Field& field) noexcept {
    /// List of all factors that combined results in an evaluation how good the play is.
    scoring = {{PlayEvaluator::getGlobalEvaluation(eval::BallOnTheirSide, world), 1.0}};
    return (lastScore = PlayEvaluator::calculateScore(scoring)).value();
}

Dealer::FlagMap AggressiveStopFormation::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {}}});
    flagMap.insert({"formation_back_0", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"formation_back_1", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"formation_back_2", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"formation_mid_0", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_mid_1", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_mid_2", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_front_0", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"formation_front_1", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"formation_front_2", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"formation_front_3", {DealerFlagPriority::HIGH_PRIORITY, {}}});

    return flagMap;
}

void AggressiveStopFormation::calculateInfoForRoles() noexcept {
    PositionComputations::calculateInfoForKeeper(stpInfos, field, world);
    PositionComputations::calculateInfoForFormation(stpInfos, roles, field, world);
}

const char* AggressiveStopFormation::getName() const { return "Aggressive Stop Formation"; }
}  // namespace rtt::ai::stp::play
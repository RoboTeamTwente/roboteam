//
// Created by timo on 3/27/20.
//

#include "stp/plays/referee_specific/DefensiveStopFormation.h"

#include "stp/roles/passive/Formation.h"

namespace rtt::ai::stp::play {

DefensiveStopFormation::DefensiveStopFormation() : Play() {
    /// Evaluations that have to be true to be considered when changing plays.
    startPlayEvaluation.clear();  // DONT TOUCH.
    startPlayEvaluation.emplace_back(eval::StopGameState);
    startPlayEvaluation.emplace_back(eval::BallOnOurSide);

    /// Evaluations that have to be true to allow the play to continue, otherwise the play will change. Plays can also end using the shouldEndPlay().
    keepPlayEvaluation.clear();  // DONT TOUCH.
    keepPlayEvaluation.emplace_back(eval::StopGameState);

    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
        // Roles is we play 6v6
        std::make_unique<role::Formation>("keeper"),
        std::make_unique<role::Formation>("formation_back_0"),
        std::make_unique<role::Formation>("formation_mid_0"),
        std::make_unique<role::Formation>("formation_front_0"),
        std::make_unique<role::Formation>("formation_back_1"),
        std::make_unique<role::Formation>("formation_mid_1"),
        // Additional roles if we play 11v11
        std::make_unique<role::Formation>("formation_front_1"),
        std::make_unique<role::Formation>("formation_back_2"),
        std::make_unique<role::Formation>("formation_mid_2"),
        std::make_unique<role::Formation>("formation_front_2"),
        std::make_unique<role::Formation>("formation_back_3"),
    };
}

uint8_t DefensiveStopFormation::score(const rtt::Field& field) noexcept {
    /// List of all factors that combined results in an evaluation how good the play is.
    scoring = {{PlayEvaluator::getGlobalEvaluation(eval::BallOnOurSide, world), 1.0}};
    return (lastScore = PlayEvaluator::calculateScore(scoring)).value();  // DONT TOUCH.
}

Dealer::FlagMap DefensiveStopFormation::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {}}});
    flagMap.insert({"formation_back_0", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"formation_back_1", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"formation_back_2", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"formation_back_3", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"formation_mid_0", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_mid_1", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_mid_2", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_front_0", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"formation_front_1", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"formation_front_2", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});

    return flagMap;
}

void DefensiveStopFormation::calculateInfoForRoles() noexcept {
    PositionComputations::calculateInfoForKeeper(stpInfos, field, world);
    PositionComputations::calculateInfoForFormation(stpInfos, roles, field, world);
}

const char* DefensiveStopFormation::getName() const { return "Defensive Stop Formation"; }
}  // namespace rtt::ai::stp::play
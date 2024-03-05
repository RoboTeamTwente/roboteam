//
// Created by jordi on 24-03-20.
//

#include "stp/plays/offensive/Attack.h"

#include <roboteam_utils/Hungarian.h>

#include "stp/computations/GoalComputations.h"
#include "stp/computations/PositionScoring.h"
#include "stp/roles/Keeper.h"
#include "stp/roles/active/Striker.h"
#include "stp/roles/passive/Defender.h"
#include "stp/roles/passive/Formation.h"

namespace rtt::ai::stp::play {

Attack::Attack() : Play() {
    // Evaluations that have to be true in order for this play to be considered valid.
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(GlobalEvaluation::NormalPlayGameState);
    startPlayEvaluation.emplace_back(GlobalEvaluation::WeHaveBall);
    startPlayEvaluation.emplace_back(GlobalEvaluation::BallNotInOurDefenseAreaAndStill);

    // Evaluations that have to be true to allow the play to continue, otherwise the play will change. Plays can also end using the shouldEndPlay().
    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(GlobalEvaluation::NormalPlayGameState);
    keepPlayEvaluation.emplace_back(GlobalEvaluation::WeHaveBall);
    keepPlayEvaluation.emplace_back(GlobalEvaluation::BallNotInOurDefenseAreaAndStill);

    // Role creation, the names should be unique. The names are used in the stpInfos-map.
    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
        // Roles is we play 6v6
        std::make_unique<role::Keeper>("keeper"),
        std::make_unique<role::Striker>("striker"),
        std::make_unique<role::Defender>("defender_0"),
        std::make_unique<role::Formation>("attacker_0"),
        std::make_unique<role::Defender>("defender_1"),
        std::make_unique<role::Formation>("attacker_1"),
        // Additional roles if we play 11v11
        std::make_unique<role::Formation>("waller_0"),
        std::make_unique<role::Formation>("waller_1"),
        std::make_unique<role::Formation>("attacker_2"),
        std::make_unique<role::Defender>("defender_2"),
        std::make_unique<role::Formation>("attacker_3"),
    };
}

uint8_t Attack::score(const rtt::Field& field) noexcept {
    // Score the position of the ball based on the odds of scoring
    return PositionScoring::scorePosition(world->getWorld()->getBall().value()->position, gen::GoalShot, field, world).score;
}

Dealer::FlagMap Attack::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER);
    Dealer::DealerFlag kickerFlag(DealerFlagTitle::CAN_KICK_BALL);
    Dealer::DealerFlag detectionFlag(DealerFlagTitle::CAN_DETECT_BALL);

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {keeperFlag}}});
    flagMap.insert({"striker", {DealerFlagPriority::REQUIRED, {kickerFlag, detectionFlag}}});
    flagMap.insert({"defender_0", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"defender_1", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"defender_2", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"waller_0", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"waller_1", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"attacker_0", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"attacker_1", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"attacker_2", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"attacker_3", {DealerFlagPriority::HIGH_PRIORITY, {}}});

    return flagMap;
}

void Attack::calculateInfoForRoles() noexcept {
    PositionComputations::calculateInfoForDefendersAndWallers(stpInfos, roles, field, world, true);
    PositionComputations::calculateInfoForAttackers(stpInfos, roles, field, world);

    // Striker
    auto goalTarget = computations::GoalComputations::calculateGoalTarget(world, field);
    goalTarget.y = std::clamp(goalTarget.y, field.rightGoalArea.bottom() + 0.4, field.rightGoalArea.top() - 0.4);
    stpInfos["striker"].setPositionToShootAt(goalTarget);
    stpInfos["striker"].setKickOrChip(KickOrChip::KICK);
    stpInfos["striker"].setShotType(ShotType::MAX);
    PositionComputations::recalculateInfoForNonPassers(stpInfos, field, world, goalTarget);
}

bool Attack::shouldEndPlay() noexcept {
    return std::any_of(roles.begin(), roles.end(), [](const std::unique_ptr<Role>& role) { return role != nullptr && role->getName() == "striker" && role->finished(); });
}

const char* Attack::getName() const { return "Attack"; }

}  // namespace rtt::ai::stp::play

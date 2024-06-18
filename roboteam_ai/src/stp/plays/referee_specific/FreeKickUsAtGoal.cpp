#include "stp/plays/referee_specific/FreeKickUsAtGoal.h"

#include "stp/computations/GoalComputations.h"
#include "stp/computations/PositionScoring.h"
#include "stp/roles/Keeper.h"
#include "stp/roles/active/FreeKickTaker.h"
#include "stp/roles/passive/Defender.h"
#include "stp/roles/passive/Formation.h"

namespace rtt::ai::stp::play {

FreeKickUsAtGoal::FreeKickUsAtGoal() : Play() {
    // Evaluations that have to be true in order for this play to be considered valid.
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(GlobalEvaluation::FreeKickUsGameState);

    // Evaluations that have to be true to allow the play to continue, otherwise the play will change. Plays can also end using the shouldEndPlay().
    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(GlobalEvaluation::FreeKickUsGameState);

    // Role creation, the names should be unique. The names are used in the stpInfos-map.
    roles = std::array<std::unique_ptr<Role>, rtt::ai::constants::MAX_ROBOT_COUNT>{
        // Roles is we play 6v6
        std::make_unique<role::Keeper>("keeper"),
        std::make_unique<role::FreeKickTaker>("free_kick_taker"),
        std::make_unique<role::Defender>("defender_0"),
        std::make_unique<role::Defender>("defender_1"),
        std::make_unique<role::Defender>("waller_0"),
        std::make_unique<role::Defender>("defender_2"),
        // Additional roles if we play 11v11
        std::make_unique<role::Defender>("defender_3"),
        std::make_unique<role::Formation>("attacker_0"),
        std::make_unique<role::Defender>("waller_1"),
        std::make_unique<role::Defender>("defender_4"),
        std::make_unique<role::Defender>("defender_5"),
    };
}

uint8_t FreeKickUsAtGoal::score(const rtt::Field& field) noexcept {
    // If we are in the FreeKickUsAtGoal gameState, we always want to execute this play
    return PositionScoring::scorePosition(world->getWorld()->getBall().value()->position, gen::GoalShot, field, world).score * (rand() % (2) + 1);
}

Dealer::FlagMap FreeKickUsAtGoal::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;

    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER);
    Dealer::DealerFlag kickerFlag(DealerFlagTitle::CAN_KICK_BALL);
    Dealer::DealerFlag detectionFlag(DealerFlagTitle::CAN_DETECT_BALL);

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {keeperFlag}}});
    flagMap.insert({"free_kick_taker", {DealerFlagPriority::REQUIRED, {kickerFlag, detectionFlag}}});
    flagMap.insert({"defender_0", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"defender_1", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"defender_2", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"defender_3", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"defender_4", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"defender_5", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"waller_0", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"waller_1", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"attacker_0", {DealerFlagPriority::LOW_PRIORITY, {}}});

    return flagMap;
}

void FreeKickUsAtGoal::calculateInfoForRoles() noexcept {
    PositionComputations::calculateInfoForDefendersAndWallers(stpInfos, roles, field, world, true);
    PositionComputations::calculateInfoForAttackers(stpInfos, roles, field, world);

    // FreeKickTaker
    auto goalTarget = computations::GoalComputations::calculateGoalTarget(world, field);
    stpInfos["free_kick_taker"].setPositionToShootAt(goalTarget);
    stpInfos["free_kick_taker"].setKickOrChip(KickOrChip::KICK);
    stpInfos["free_kick_taker"].setShotType(ShotType::MAX);
}

bool FreeKickUsAtGoal::shouldEndPlay() noexcept {
    return std::any_of(roles.begin(), roles.end(), [](const std::unique_ptr<Role>& role) {
        return role != nullptr && role->getName() == "free_kick_taker" && strcmp(role->getCurrentTactic()->getName(), "Formation") == 0;
    });
}

const char* FreeKickUsAtGoal::getName() const { return "Free Kick Us At Goal"; }

}  // namespace rtt::ai::stp::play

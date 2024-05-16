#include "stp/plays/referee_specific/FreeKickThem.h"

#include "stp/computations/PositionComputations.h"
#include "stp/roles/Keeper.h"
#include "stp/roles/active/Harasser.h"
#include "stp/roles/passive/Defender.h"
#include "stp/roles/passive/Formation.h"

namespace rtt::ai::stp::play {

FreeKickThem::FreeKickThem() : Play() {
    // Evaluations that have to be true in order for this play to be considered valid.
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(GlobalEvaluation::FreeKickThemGameState);

    // Evaluations that have to be true to allow the play to continue, otherwise the play will change. Plays can also end using the shouldEndPlay().
    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(GlobalEvaluation::FreeKickThemGameState);

    // Role creation, the names should be unique. The names are used in the stpInfos-map.
    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
        // Roles is we play 6v6
        std::make_unique<role::Keeper>("keeper"),
        std::make_unique<role::Harasser>("harasser"),
        std::make_unique<role::Formation>("waller_0"),
        std::make_unique<role::Formation>("waller_1"),
        std::make_unique<role::Defender>("defender_0"),
        std::make_unique<role::Defender>("defender_1"),
        // Additional roles if we play 11v11
        std::make_unique<role::Defender>("defender_2"),
        std::make_unique<role::Formation>("waller_2"),
        std::make_unique<role::Formation>("attacker_0"),
        std::make_unique<role::Formation>("waller_3"),
        std::make_unique<role::Defender>("defender_3"),
    };
}

uint8_t FreeKickThem::score(const rtt::Field&) noexcept {
    // If this play is valid we always want to execute this play
    return control_constants::FUZZY_TRUE;
}

Dealer::FlagMap FreeKickThem::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;

    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER);

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {keeperFlag}}});
    flagMap.insert({"harasser", {DealerFlagPriority::REQUIRED, {}}});
    flagMap.insert({"waller_0", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"waller_1", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"waller_2", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"waller_3", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"defender_0", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"defender_1", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"defender_2", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"defender_3", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"attacker_0", {DealerFlagPriority::LOW_PRIORITY, {}}});

    return flagMap;
}

void FreeKickThem::calculateInfoForRoles() noexcept {
    PositionComputations::calculateInfoForDefendersAndWallers(stpInfos, roles, field, world, false);
    PositionComputations::calculateInfoForAttackers(stpInfos, roles, field, world);
    calculateInfoForHarasser();
}

void FreeKickThem::calculateInfoForHarasser() noexcept {
    auto ballPos = world->getWorld()->getBall()->get()->position;
    auto enemyClosestToBallOpt = world->getWorld()->getRobotClosestToBall(world::Team::them);
    if (!enemyClosestToBallOpt) return;
    auto enemyClosestToBall = enemyClosestToBallOpt.value();

    auto enemyToBall = (ballPos - enemyClosestToBall->getPos());
    auto targetPos = ballPos + (enemyToBall).stretchToLength(control_constants::AVOID_BALL_DISTANCE);
    stpInfos["harasser"].setPositionToMoveTo(targetPos);
    stpInfos["harasser"].setAngle(enemyToBall.angle() + M_PI);
}

const char* FreeKickThem::getName() const { return "Free Kick Them"; }
}  // namespace rtt::ai::stp::play

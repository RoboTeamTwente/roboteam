//
// Created by jessevw on 24.03.20.
//

#include "stp/plays/referee_specific/BallPlacementThem.h"

#include "stp/roles/Keeper.h"
#include "stp/roles/passive/Defender.h"
#include "stp/roles/passive/Formation.h"

namespace rtt::ai::stp::play {

BallPlacementThem::BallPlacementThem() : Play() {
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(eval::BallPlacementThemGameState);

    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(eval::BallPlacementThemGameState);

    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
        // Roles is we play 6v6
        std::make_unique<role::Formation>("keeper"),
        std::make_unique<role::Formation>("harasser"),
        std::make_unique<role::Formation>("waller_0"),
        std::make_unique<role::Formation>("waller_1"),
        std::make_unique<role::Defender>("defender_0"),
        std::make_unique<role::Defender>("defender_1"),
        // Additional roles if we play 11v11
        std::make_unique<role::Formation>("waller_2"),
        std::make_unique<role::Defender>("defender_2"),
        std::make_unique<role::Defender>("defender_3"),
        std::make_unique<role::Formation>("waller_3"),
        std::make_unique<role::Formation>("attacker_0"),
    };
}

uint8_t BallPlacementThem::score(const rtt::Field& field) noexcept {
    // If this play is valid we always want to execute this play
    return control_constants::FUZZY_TRUE;
}

Dealer::FlagMap BallPlacementThem::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER);

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {keeperFlag}}});
    flagMap.insert({"harasser", {DealerFlagPriority::REQUIRED, {}}});
    flagMap.insert({"waller_0", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"waller_1", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"waller_2", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"waller_3", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"defender_0", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"defender_1", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"defender_2", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"defender_3", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"attacker_0", {DealerFlagPriority::LOW_PRIORITY, {}}});

    return flagMap;
}

void BallPlacementThem::calculateInfoForRoles() noexcept {
    PositionComputations::calculateInfoForKeeper(stpInfos, field, world);
    PositionComputations::calculateInfoForDefendersAndWallers(stpInfos, roles, field, world);
    PositionComputations::calculateInfoForAttackers(stpInfos, roles, field, world);
    calculateInfoForHarasser();
}

void BallPlacementThem::calculateInfoForHarasser() noexcept {
    auto placementPos = rtt::ai::GameStateManager::getRefereeDesignatedPosition();
    auto targetPos = placementPos + (field.leftGoalArea.rightLine().center() - placementPos).stretchToLength(control_constants::AVOID_BALL_DISTANCE);
    stpInfos["harasser"].setPositionToMoveTo(targetPos);
    stpInfos["harasser"].setAngle((placementPos - field.leftGoalArea.rightLine().center()).toAngle());
}

const char* BallPlacementThem::getName() const { return "Ball Placement Them"; }
}  // namespace rtt::ai::stp::play

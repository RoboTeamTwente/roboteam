//
// Created by jessevw on 24.03.20.
//

#include "stp/plays/referee_specific/BallPlacementThem.h"

#include "stp/roles/Keeper.h"
#include "stp/roles/passive/BallAvoider.h"
#include "stp/roles/passive/BallDefender.h"

namespace rtt::ai::stp::play {

BallPlacementThem::BallPlacementThem() : Play() {
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(eval::BallPlacementThemGameState);

    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(eval::BallPlacementThemGameState);

    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
        // Roles is we play 6v6
        std::make_unique<role::BallAvoider>("keeper"),
        std::make_unique<role::BallAvoider>("harasser"),
        std::make_unique<role::BallAvoider>("waller_0"),
        std::make_unique<role::BallAvoider>("waller_1"),
        std::make_unique<role::BallDefender>("defender_0"),
        std::make_unique<role::BallDefender>("defender_1"),
        // Additional roles if we play 11v11
        std::make_unique<role::BallAvoider>("waller_2"),
        std::make_unique<role::BallDefender>("defender_2"),
        std::make_unique<role::BallDefender>("defender_3"),
        std::make_unique<role::BallAvoider>("waller_3"),
        std::make_unique<role::BallAvoider>("attacker_0"),
    };
}

uint8_t BallPlacementThem::score(const rtt::Field& field) noexcept {
    /// List of all factors that combined results in an evaluation how good the play is.
    scoring = {{PlayEvaluator::getGlobalEvaluation(eval::BallPlacementThemGameState, world), 1.0}};
    return (lastScore = PlayEvaluator::calculateScore(scoring)).value();  // DONT TOUCH.
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
    PositionComputations::calculateInfoForWallers(stpInfos, roles, field, world);
    PositionComputations::calculateInfoForDefenders(stpInfos, roles, field, world);
    PositionComputations::calculateInfoForAttackers(stpInfos, roles, field, world);
    calculateInfoForHarasser();
}

void BallPlacementThem::calculateInfoForHarasser() noexcept {
    auto placementPos = rtt::ai::GameStateManager::getRefereeDesignatedPosition();
    auto targetPos = placementPos + (field.leftGoalArea.rightLine().center() - placementPos).stretchToLength(control_constants::AVOID_BALL_DISTANCE);
    targetPos = PositionComputations::calculateAvoidBallPosition(targetPos, world->getWorld()->getBall().value()->position, field);
    stpInfos["harasser"].setPositionToMoveTo(targetPos);
    stpInfos["harasser"].setAngle((placementPos - field.leftGoalArea.rightLine().center()).toAngle());
}

const char* BallPlacementThem::getName() const { return "Ball Placement Them"; }
}  // namespace rtt::ai::stp::play

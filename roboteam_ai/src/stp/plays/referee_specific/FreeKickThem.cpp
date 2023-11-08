//
// Created by jordi on 07-05-20.
//

#include "stp/plays/referee_specific/FreeKickThem.h"

#include "stp/computations/PositionComputations.h"
#include "stp/roles/Keeper.h"
#include "stp/roles/active/Harasser.h"
#include "stp/roles/passive/BallDefender.h"
#include "stp/roles/passive/Formation.h"

namespace rtt::ai::stp::play {

FreeKickThem::FreeKickThem() : Play() {
    /// Evaluations that have to be true to be considered when changing plays.
    startPlayEvaluation.clear();  // DONT TOUCH.
    startPlayEvaluation.emplace_back(eval::FreeKickThemGameState);

    /// Evaluations that have to be true to allow the play to continue, otherwise the play will change. Plays can also end using the shouldEndPlay().
    keepPlayEvaluation.clear();  // DONT TOUCH.
    keepPlayEvaluation.emplace_back(eval::FreeKickThemGameState);

    roles = std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT>{
        // Roles is we play 6v6
        std::make_unique<role::Keeper>("keeper"),
        std::make_unique<role::Harasser>("harasser"),
        std::make_unique<role::Formation>("waller_0"),
        std::make_unique<role::Formation>("waller_1"),
        std::make_unique<role::BallDefender>("defender_0"),
        std::make_unique<role::BallDefender>("defender_1"),
        // Additional roles if we play 11v11
        std::make_unique<role::BallDefender>("defender_2"),
        std::make_unique<role::Formation>("waller_2"),
        std::make_unique<role::Formation>("attacker_0"),
        std::make_unique<role::Formation>("waller_3"),
        std::make_unique<role::BallDefender>("defender_3"),
    };
}

uint8_t FreeKickThem::score(const rtt::Field& field) noexcept {
    /// List of all factors that combined results in an evaluation how good the play is.
    scoring = {{PlayEvaluator::getGlobalEvaluation(eval::FreeKickThemGameState, world), 1.0}};
    return (lastScore = PlayEvaluator::calculateScore(scoring)).value();  // DONT TOUCH.
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
    PositionComputations::calculateInfoForKeeper(stpInfos, field, world);
    PositionComputations::calculateInfoForDefendersAndWallers(stpInfos, roles, field, world);
    PositionComputations::calculateInfoForAttackers(stpInfos, roles, field, world);
    calculateInfoForHarasser();
}

void FreeKickThem::calculateInfoForHarasser() noexcept {
    auto placementPos = rtt::ai::GameStateManager::getRefereeDesignatedPosition();
    auto enemyClosestToBallOpt = world->getWorld()->getRobotClosestToBall(world::Team::them);
    if (!enemyClosestToBallOpt) return;
    auto enemyClosestToBall = enemyClosestToBallOpt.value();

    auto enemyToBall = (world->getWorld()->getBall()->get()->position - enemyClosestToBall->getPos());
    auto targetPos = placementPos + (enemyToBall).stretchToLength(control_constants::AVOID_BALL_DISTANCE);

    // Make sure this is not too close to the ball
    targetPos = PositionComputations::calculateAvoidBallPosition(targetPos, world->getWorld()->getBall().value()->position, field);
    stpInfos["harasser"].setPositionToMoveTo(targetPos);
    stpInfos["harasser"].setAngle(enemyToBall.angle() + M_PI);
}

const char* FreeKickThem::getName() const { return "Free Kick Them"; }
}  // namespace rtt::ai::stp::play

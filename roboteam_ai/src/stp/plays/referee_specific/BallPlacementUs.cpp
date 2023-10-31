//
// Created by jessevw on 24.03.20.
//

#include "stp/plays/referee_specific/BallPlacementUs.h"

#include "stp/roles/active/BallPlacer.h"
#include "stp/roles/passive/BallAvoider.h"
#include "utilities/GameStateManager.hpp"

namespace rtt::ai::stp::play {

BallPlacementUs::BallPlacementUs() : Play() {
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(eval::BallPlacementUsGameState);

    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(eval::BallPlacementUsGameState);

    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
        // Roles is we play 6v6
        std::make_unique<role::BallAvoider>("keeper"), std::make_unique<role::BallPlacer>("ball_placer"), std::make_unique<role::BallAvoider>("formation_back_0"),
        std::make_unique<role::BallAvoider>("formation_mid_0"), std::make_unique<role::BallAvoider>("formation_front_0"), std::make_unique<role::BallAvoider>("formation_mid_1"),
        // Additional roles if we play 11v11
        std::make_unique<role::BallAvoider>("formation_back_1"), std::make_unique<role::BallAvoider>("formation_front_1"), std::make_unique<role::BallAvoider>("formation_back_2"),
        std::make_unique<role::BallAvoider>("formation_mid_2"), std::make_unique<role::BallAvoider>("formation_front_2")};
}

uint8_t BallPlacementUs::score(const rtt::Field& field) noexcept {
    /// List of all factors that combined results in an evaluation how good the play is.
    scoring = {{PlayEvaluator::getGlobalEvaluation(eval::BallPlacementUsGameState, world), 1.0}};
    return (lastScore = PlayEvaluator::calculateScore(scoring)).value();
}

Dealer::FlagMap BallPlacementUs::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag detectionFlag(DealerFlagTitle::CAN_DETECT_BALL);
    Dealer::DealerFlag dribblerFlag(DealerFlagTitle::WITH_WORKING_DRIBBLER);

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {}}});
    flagMap.insert({"ball_placer", {DealerFlagPriority::REQUIRED, {dribblerFlag, detectionFlag}}});
    flagMap.insert({"formation_back_0", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_back_1", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_back_2", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_mid_0", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_mid_1", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_mid_2", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_front_0", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_front_1", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_front_2", {DealerFlagPriority::LOW_PRIORITY, {}}});

    return flagMap;
}

void BallPlacementUs::calculateInfoForRoles() noexcept {
    PositionComputations::calculateInfoForKeeper(stpInfos, field, world);
    PositionComputations::calculateInfoForFormation(stpInfos, roles, field, world);

    Vector2 ballTarget = Vector2();

    // Adjust placement position to be one robot radius away in the distance of movement
    if (stpInfos["ball_placer"].getRobot()) {
        ballTarget = rtt::ai::GameStateManager::getRefereeDesignatedPosition();
        ballTarget -= (world->getWorld()->get()->getBall()->get()->position - stpInfos["ball_placer"].getRobot()->get()->getPos()).stretchToLength(control_constants::ROBOT_RADIUS);
    } else {
        // If we don't have a ball placer, set the target location to the ball, such that the dealer will
        // assign the robot closest to the ball to the ball placer role
        ballTarget = world->getWorld()->get()->getBall()->get()->position;
    }

    stpInfos["ball_placer"].setPositionToShootAt(ballTarget);
    stpInfos["ball_placer"].setPositionToMoveTo(ballTarget);
    stpInfos["ball_placer"].setShouldAvoidDefenseArea(false);
    stpInfos["ball_placer"].setShouldAvoidOutOfField(false);

    if (stpInfos["ball_placer"].getRobot() && stpInfos["ball_placer"].getRobot()->get()->getDistanceToBall() < 1.0) stpInfos["ball_placer"].setMaxRobotVelocity(0.75);
    if (stpInfos["ball_placer"].getRobot() && stpInfos["ball_placer"].getRobot()->get()->getDistanceToBall() < control_constants::TURN_ON_DRIBBLER_DISTANCE) {
        stpInfos["ball_placer"].setDribblerSpeed(100);
    }
}

const char* BallPlacementUs::getName() const { return "Ball Placement Us"; }
}  // namespace rtt::ai::stp::play

#include "stp/plays/referee_specific/BallPlacementUsForceStart.h"

#include "stp/roles/Keeper.h"
#include "stp/roles/active/BallPlacer.h"
#include "stp/roles/passive/Defender.h"
#include "stp/roles/passive/Formation.h"
#include "utilities/GameStateManager.hpp"
#include "STPManager.h"
#include "utilities/Constants.h"

namespace rtt::ai::stp::play {

BallPlacementUsForceStart::BallPlacementUsForceStart() : Play() {
    // Evaluations that have to be true in order for this play to be considered valid.
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(GlobalEvaluation::BallPlacementUsGameState);

    // Evaluations that have to be true to allow the play to continue
    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(GlobalEvaluation::BallPlacementUsGameState);

    updateRoleConfiguration();
}

uint8_t BallPlacementUsForceStart::score(const rtt::Field&) noexcept {
    // If this play is valid we always want to execute this play
    return constants::FUZZY_TRUE;
}

Dealer::FlagMap BallPlacementUsForceStart::decideRoleFlags() const noexcept {
    const_cast<BallPlacementUsForceStart*>(this)->updateRoleConfiguration();
    
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag detectionFlag(DealerFlagTitle::CAN_DETECT_BALL);
    Dealer::DealerFlag dribblerFlag(DealerFlagTitle::WITH_WORKING_DRIBBLER);
    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER);

    // Required roles with specific priorities
    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {keeperFlag}}});
    flagMap.insert({"ball_placer", {DealerFlagPriority::REQUIRED, {dribblerFlag, detectionFlag}}});

    // Add wallers with LOW_PRIORITY
    for (int i = 0; i < numWallers; i++) {
        flagMap.insert({"waller_" + std::to_string(i), {DealerFlagPriority::LOW_PRIORITY, {}}});
    }

    // Add defenders with MEDIUM_PRIORITY
    for (int i = 0; i < numDefenders; i++) {
        flagMap.insert({"defender_" + std::to_string(i), {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    }

    // Add attackers with HIGH_PRIORITY
    for (int i = 0; i < numAttackers; i++) {
        flagMap.insert({"attacker_" + std::to_string(i), {DealerFlagPriority::HIGH_PRIORITY, {}}});
    }

    return flagMap;
}

void BallPlacementUsForceStart::calculateInfoForRoles() noexcept {
    PositionComputations::calculateInfoForDefendersAndWallers(stpInfos, roles, field, world, false);
    PositionComputations::calculateInfoForAttackers(stpInfos, roles, field, world);

    Vector2 ballTarget;

    // Adjust placement position to be one robot radius away in the distance of movement
    if (stpInfos["ball_placer"].getRobot()) {
        ballTarget = rtt::ai::GameStateManager::getRefereeDesignatedPosition();
        ballTarget -= (world->getWorld()->get()->getBall()->get()->position - stpInfos["ball_placer"].getRobot()->get()->getPos()).stretchToLength(constants::ROBOT_RADIUS);
    } else {
        // If we don't have a ball placer, set the target location to the ball, such that the dealer will
        // assign the robot closest to the ball to the ball placer role
        ballTarget = world->getWorld()->get()->getBall()->get()->position;
    }

    for (auto& stpInfo : stpInfos) {
        stpInfo.second.setShouldAvoidOurDefenseArea(false);
        stpInfo.second.setShouldAvoidTheirDefenseArea(false);
        stpInfo.second.setShouldAvoidBall(true);
    }
    stpInfos["ball_placer"].setPositionToMoveTo(ballTarget);
    stpInfos["ball_placer"].setShouldAvoidOutOfField(false);
    stpInfos["ball_placer"].setShouldAvoidBall(false);

    if (stpInfos["ball_placer"].getRobot() && stpInfos["ball_placer"].getRobot()->get()->hasBall()) stpInfos["ball_placer"].setMaxRobotVelocity(0.75);
    if (stpInfos["ball_placer"].getRobot() && stpInfos["ball_placer"].getRobot()->get()->getDistanceToBall() < constants::TURN_ON_DRIBBLER_DISTANCE) {
        stpInfos["ball_placer"].setDribblerOn(true);
    }
}

void BallPlacementUsForceStart::updateRoleConfiguration() {
    if (STPManager::isInitialized() && STPManager::getRLInterface().getIsActive()) {
        // Calculate required number of wallers based on ball position and angles
        PositionComputations::setAmountOfWallers(field, world);
        
        int availableSlots = rtt::ai::constants::MAX_ROBOT_COUNT - MANDATORY_ROLES;
        
        // Get number of attackers from RL
        numAttackers = STPManager::getRLInterface().getNumAttackers();
        numAttackers = std::min(numAttackers, availableSlots);
        availableSlots -= numAttackers;
        
        // Assign wallers based on the dynamic computation
        numWallers = std::min(PositionComputations::amountOfWallers, availableSlots);
        availableSlots -= numWallers;
        
        // Use remaining slots for defenders
        numDefenders = availableSlots;
    }
    
    // Create roles array
    roles = std::array<std::unique_ptr<Role>, rtt::ai::constants::MAX_ROBOT_COUNT>();

    // Create mandatory roles first
    roles[0] = std::make_unique<role::Keeper>("keeper");
    roles[1] = std::make_unique<role::BallPlacer>("ball_placer");

    int currentIndex = MANDATORY_ROLES;

    // Add wallers
    for (int i = 0; i < numWallers && currentIndex < rtt::ai::constants::MAX_ROBOT_COUNT; i++) {
        roles[currentIndex++] = std::make_unique<role::Defender>("waller_" + std::to_string(i));
    }

    // Add defenders
    for (int i = 0; i < numDefenders && currentIndex < rtt::ai::constants::MAX_ROBOT_COUNT; i++) {
        roles[currentIndex++] = std::make_unique<role::Defender>("defender_" + std::to_string(i));
    }

    // Add attackers
    for (int i = 0; i < numAttackers && currentIndex < rtt::ai::constants::MAX_ROBOT_COUNT; i++) {
        roles[currentIndex++] = std::make_unique<role::Formation>("attacker_" + std::to_string(i));
    }
}

const char* BallPlacementUsForceStart::getName() const { return "Ball Placement Us Force Start"; }

}  // namespace rtt::ai::stp::play
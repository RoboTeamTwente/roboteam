#include "stp/plays/referee_specific/BallPlacementThem.h"

#include "stp/roles/Keeper.h"
#include "stp/roles/passive/Defender.h"
#include "stp/roles/passive/Formation.h"
#include "STPManager.h"
#include "utilities/Constants.h"

namespace rtt::ai::stp::play {

BallPlacementThem::BallPlacementThem() : Play() {
    // Evaluations that have to be true in order for this play to be considered valid.
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(GlobalEvaluation::BallPlacementThemGameState);

    // Evaluations that have to be true to allow the play to continue
    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(GlobalEvaluation::BallPlacementThemGameState);

    updateRoleConfiguration();
}

uint8_t BallPlacementThem::score(const rtt::Field&) noexcept {
    // If this play is valid we always want to execute this play
    return constants::FUZZY_TRUE;
}

Dealer::FlagMap BallPlacementThem::decideRoleFlags() const noexcept {
    const_cast<BallPlacementThem*>(this)->updateRoleConfiguration();
    
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER);

    // Required roles with specific priorities
    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {keeperFlag}}});
    flagMap.insert({"harasser", {DealerFlagPriority::REQUIRED, {}}});

    // Add wallers with dynamic priority
    for (int i = 0; i < numWallers; i++) {
        flagMap.insert({"waller_" + std::to_string(i), {DealerFlagPriority::HIGH_PRIORITY, {}}});
    }

    // Add defenders
    for (int i = 0; i < numDefenders; i++) {
        flagMap.insert({"defender_" + std::to_string(i), {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    }

    // Add attackers
    for (int i = 0; i < numAttackers; i++) {
        flagMap.insert({"attacker_" + std::to_string(i), {DealerFlagPriority::LOW_PRIORITY, {}}});
    }

    return flagMap;
}

void BallPlacementThem::calculateInfoForRoles() noexcept {
    PositionComputations::calculateInfoForDefendersAndWallers(stpInfos, roles, field, world, false);
    PositionComputations::calculateInfoForAttackers(stpInfos, roles, field, world);
    calculateInfoForHarasser();
    for (auto& stpInfo : stpInfos) {
        stpInfo.second.setShouldAvoidOurDefenseArea(false);
        stpInfo.second.setShouldAvoidTheirDefenseArea(false);
    }
}

void BallPlacementThem::calculateInfoForHarasser() noexcept {
    auto placementPos = rtt::ai::GameStateManager::getRefereeDesignatedPosition();
    auto targetPos = placementPos + (field.leftGoalArea.rightLine().center() - placementPos).stretchToLength(constants::AVOID_BALL_DISTANCE);
    stpInfos["harasser"].setPositionToMoveTo(targetPos);
    stpInfos["harasser"].setYaw((placementPos - field.leftGoalArea.rightLine().center()).toAngle());
}

void BallPlacementThem::updateRoleConfiguration() {
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
    roles[1] = std::make_unique<role::Formation>("harasser");

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

const char* BallPlacementThem::getName() const { return "Ball Placement Them"; }

}  // namespace rtt::ai::stp::play
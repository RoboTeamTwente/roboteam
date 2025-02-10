#include "stp/plays/offensive/Attack.h"

#include <roboteam_utils/Hungarian.h>

#include "stp/computations/GoalComputations.h"
#include "stp/computations/PositionScoring.h"
#include "stp/roles/Keeper.h"
#include "stp/roles/active/Striker.h"
#include "stp/roles/passive/Defender.h"
#include "stp/roles/passive/Formation.h"
#include "STPManager.h"
#include "utilities/Constants.h"

namespace rtt::ai::stp::play {

Attack::Attack() : Play() {
    // Evaluations that have to be true in order for this play to be considered valid.
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(GlobalEvaluation::NormalPlayGameState);
    startPlayEvaluation.emplace_back(GlobalEvaluation::WeWillHaveBall);
    startPlayEvaluation.emplace_back(GlobalEvaluation::BallNotInOurDefenseAreaAndStill);

    // Evaluations that have to be true to allow the play to continue
    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(GlobalEvaluation::NormalPlayGameState);
    keepPlayEvaluation.emplace_back(GlobalEvaluation::WeWillHaveBall);
    keepPlayEvaluation.emplace_back(GlobalEvaluation::BallNotInOurDefenseAreaAndStill);

    updateRoleConfiguration();
}

uint8_t Attack::score(const rtt::Field& field) noexcept {
    // Score the position of the ball based on the odds of scoring
    return PositionScoring::scorePosition(world->getWorld()->getBall().value()->position, gen::GoalShot, field, world).score * (rand() % (2) + 1);
}

Dealer::FlagMap Attack::decideRoleFlags() const noexcept {
    const_cast<Attack*>(this)->updateRoleConfiguration();
    
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER);
    Dealer::DealerFlag kickerFlag(DealerFlagTitle::CAN_KICK_BALL);
    Dealer::DealerFlag detectionFlag(DealerFlagTitle::CAN_DETECT_BALL);

    // Required roles with specific priorities
    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {keeperFlag}}});
    flagMap.insert({"striker", {DealerFlagPriority::REQUIRED, {kickerFlag, detectionFlag}}});

    // Add wallers with dynamic priority
    for (int i = 0; i < numWallers; i++) {
        if (i <= PositionComputations::amountOfWallers) {
            flagMap.insert({"waller_" + std::to_string(i), {DealerFlagPriority::HIGH_PRIORITY, {}}});
        } else {
            flagMap.insert({"waller_" + std::to_string(i), {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
        }
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

void Attack::calculateInfoForRoles() noexcept {
    PositionComputations::calculateInfoForDefendersAndWallers(stpInfos, roles, field, world, true);
    PositionComputations::calculateInfoForAttackers(stpInfos, roles, field, world);

    // Striker
    auto goalTarget = computations::GoalComputations::calculateGoalTarget(world, field);
    goalTarget.y = std::clamp(goalTarget.y, field.rightGoalArea.bottom() + 0.2, field.rightGoalArea.top() - 0.2);
    stpInfos["striker"].setPositionToShootAt(goalTarget);
    stpInfos["striker"].setKickOrChip(KickType::KICK);
    stpInfos["striker"].setShotPower(ShotPower::MAX);
    PositionComputations::recalculateInfoForNonPassers(stpInfos, field, world, goalTarget);
}

void Attack::updateRoleConfiguration() {
    if (STPManager::isInitialized() && STPManager::getRLInterface().getIsActive()) {
        // Get suggested number of attackers from RL
        int availableSlots = rtt::ai::constants::MAX_ROBOT_COUNT - MANDATORY_ROLES;
        
        // Get and cap number of attackers
        numAttackers = STPManager::getRLInterface().getNumAttackers();
        numAttackers = std::min(numAttackers, availableSlots);
        availableSlots -= numAttackers;
        
        // Distribute remaining slots between wallers and defenders
        numWallers = std::min(2, availableSlots);  // Always try to have 2 wallers if possible
        availableSlots -= numWallers;
        
        numDefenders = availableSlots;  // Use remaining slots for defenders
    }
    
    // Create roles array
    roles = std::array<std::unique_ptr<Role>, rtt::ai::constants::MAX_ROBOT_COUNT>();

    // Create mandatory roles first
    roles[0] = std::make_unique<role::Keeper>("keeper");
    roles[1] = std::make_unique<role::Striker>("striker");

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

bool Attack::shouldEndPlay() noexcept {
    // If the striker has finished, the play finished successfully
    if (std::any_of(roles.begin(), roles.end(), [](const auto& role) { return role != nullptr && role->getName() == "striker" && role->finished(); })) {
        return true;
    }

    // Find id of robot with name striker
    auto strikerId = stpInfos.at("striker");
    if (strikerId.getRobot() && strikerId.getRobot().value()) {
        auto strikerRobotId = strikerId.getRobot().value()->getId();
        if (InterceptionComputations::calculateInterceptionInfoExcludingKeeperAndCarded(world).interceptId != strikerRobotId) {
            return true;
        }
    }

    return false;
}

const char* Attack::getName() const { return "Attack"; }

}  // namespace rtt::ai::stp::play
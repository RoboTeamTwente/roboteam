#include "stp/plays/defensive/Defend.h"
#include "stp/roles/Keeper.h"
#include "stp/roles/active/Harasser.h"
#include "stp/roles/passive/Defender.h"
#include "stp/roles/passive/Formation.h"
#include "STPManager.h"
#include "rl/RLInterface.hpp"

namespace rtt::ai::stp::play {

Defend::Defend() : Play() {
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(GlobalEvaluation::NormalPlayGameState);
    startPlayEvaluation.emplace_back(GlobalEvaluation::WeWillNotHaveBall);
    startPlayEvaluation.emplace_back(GlobalEvaluation::BallNotInOurDefenseAreaAndStill);

    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(GlobalEvaluation::NormalPlayGameState);
    keepPlayEvaluation.emplace_back(GlobalEvaluation::TheyHaveBall);
    keepPlayEvaluation.emplace_back(GlobalEvaluation::BallNotInOurDefenseAreaAndStill);

    updateRoleConfiguration();
}

uint8_t Defend::score(const rtt::Field&) noexcept {
    return constants::FUZZY_TRUE;
}

Dealer::FlagMap Defend::decideRoleFlags() const noexcept {
    const_cast<Defend*>(this)->updateRoleConfiguration();
    
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER);

    // Required roles
    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {keeperFlag}}});
    flagMap.insert({"harasser", {DealerFlagPriority::REQUIRED, {}, harasserInfo.interceptId}});

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
        flagMap.insert({"attacker_" + std::to_string(i), {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    }

    return flagMap;
}

void Defend::calculateInfoForRoles() noexcept {
    harasserInfo = InterceptionComputations::calculateInterceptionInfoExcludingKeeperAndCarded(world);
    PositionComputations::calculateInfoForHarasser(stpInfos, &roles, field, world, harasserInfo.interceptLocation);
    PositionComputations::calculateInfoForDefendersAndWallers(stpInfos, roles, field, world, false);
    PositionComputations::calculateInfoForAttackers(stpInfos, roles, field, world);
}

void Defend::updateRoleConfiguration() {
    
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
    roles[1] = std::make_unique<role::Harasser>("harasser");

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

const char* Defend::getName() const { return "Defend"; }

}  // namespace rtt::ai::stp::play
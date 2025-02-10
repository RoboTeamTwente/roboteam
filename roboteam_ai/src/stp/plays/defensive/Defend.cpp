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

    // Role creation, the names should be unique. The names are used in the stpInfos-map.
    roles = std::array<std::unique_ptr<Role>, rtt::ai::constants::MAX_ROBOT_COUNT>();

    // Create mandatory roles
    auto keeper = std::make_unique<role::Keeper>("keeper");
    auto harasser = std::make_unique<role::Harasser>("harasser");
    
    // Move them into the array
    roles[0] = std::move(keeper);
    roles[1] = std::move(harasser);

    int currentIndex = 2;

    // Add wallers
    for (int i = 0; i < numWallers && currentIndex < rtt::ai::constants::MAX_ROBOT_COUNT; i++) {
        auto waller = std::make_unique<role::Defender>("waller_" + std::to_string(i));
        roles[currentIndex++] = std::move(waller);
    }

    // Add defenders
    for (int i = 0; i < numDefenders && currentIndex < rtt::ai::constants::MAX_ROBOT_COUNT; i++) {
        auto defender = std::make_unique<role::Defender>("defender_" + std::to_string(i));
        roles[currentIndex++] = std::move(defender);
    }

    // Add attackers
    for (int i = 0; i < numAttackers && currentIndex < rtt::ai::constants::MAX_ROBOT_COUNT; i++) {
        auto attacker = std::make_unique<role::Formation>("attacker_" + std::to_string(i));
        roles[currentIndex++] = std::move(attacker);
    }
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
        if (i <= PositionComputations::amountOfWallers) {
            flagMap.insert({"waller_" + std::to_string(i), {DealerFlagPriority::HIGH_PRIORITY, {}}});
        } else {
            flagMap.insert({"waller_" + std::to_string(i), {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
        }
    }

    // Add defenders
    for (int i = 0; i < numDefenders; i++) {
        flagMap.insert({"defender_" + std::to_string(i), {DealerFlagPriority::HIGH_PRIORITY, {}}});
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
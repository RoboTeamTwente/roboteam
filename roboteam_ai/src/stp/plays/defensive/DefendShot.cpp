//
// Created by agata on 14/01/2022.
//

#include "stp/plays/defensive/DefendShot.h"

#include <stp/roles/passive/Formation.h>

#include "stp/roles/Keeper.h"
#include "stp/roles/active/Harasser.h"
#include "stp/roles/passive/BallDefender.h"

namespace rtt::ai::stp::play {

DefendShot::DefendShot() : Play() {
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(eval::NormalPlayGameState);
    startPlayEvaluation.emplace_back(GlobalEvaluation::BallNotInOurDefenseAreaAndStill);

    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(eval::NormalPlayGameState);
    keepPlayEvaluation.emplace_back(eval::TheyHaveBall);
    keepPlayEvaluation.emplace_back(GlobalEvaluation::BallNotInOurDefenseAreaAndStill);

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

uint8_t DefendShot::score(const rtt::Field& field) noexcept {
    if (world->getWorld()->whichRobotHasBall(world::them) != std::nullopt) return 255;
    if (world->getWorld()->whichRobotHasBall(world::us) != std::nullopt) return 0;
    return 0;
}

Dealer::FlagMap DefendShot::decideRoleFlags() const noexcept {
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

void DefendShot::calculateInfoForRoles() noexcept {
    PositionComputations::calculateInfoForKeeper(stpInfos, field, world);
    PositionComputations::calculateInfoForHarasser(stpInfos, &roles, field, world);
    PositionComputations::calculateInfoForDefendersAndWallers(stpInfos, roles, field, world);
    PositionComputations::calculateInfoForAttackers(stpInfos, roles, field, world);
}

const char* DefendShot::getName() const { return "Defend Shot"; }

// If we have the ball we should end doing defendShot
bool DefendShot::shouldEndPlay() noexcept {
    auto robotWithBall = world->getWorld()->whichRobotHasBall(world::both);
    return robotWithBall && robotWithBall->get()->getTeam() == rtt::world::Team::us;
}

}  // namespace rtt::ai::stp::play

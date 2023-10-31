//
// Created by agata on 14/01/2022.
//

#include "stp/plays/defensive/DefendPass.h"

#include <world/views/RobotView.hpp>

#include "roboteam_utils/Hungarian.h"
#include "stp/roles/Keeper.h"
#include "stp/roles/active/Harasser.h"
#include "stp/roles/passive/BallDefender.h"
#include "stp/roles/passive/Formation.h"

namespace rtt::ai::stp::play {

DefendPass::DefendPass() : Play() {
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(eval::NormalPlayGameState);
    startPlayEvaluation.emplace_back(eval::TheyHaveBall);
    startPlayEvaluation.emplace_back(GlobalEvaluation::BallNotInOurDefenseAreaAndStill);

    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(eval::NormalPlayGameState);
    keepPlayEvaluation.emplace_back(eval::TheyHaveBall);
    keepPlayEvaluation.emplace_back(GlobalEvaluation::BallNotInOurDefenseAreaAndStill);

    roles = std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT>{
        // Roles is we play 6v6
        std::make_unique<role::Keeper>("keeper"),
        std::make_unique<role::Harasser>("harasser"),
        std::make_unique<role::BallDefender>("defender_0"),
        std::make_unique<role::BallDefender>("defender_1"),
        std::make_unique<role::BallDefender>("defender_2"),
        std::make_unique<role::BallDefender>("defender_3"),
        // Additional roles if we play 11v11
        std::make_unique<role::Formation>("attacker_0"),
        std::make_unique<role::BallDefender>("defender_4"),
        std::make_unique<role::BallDefender>("defender_5"),
        std::make_unique<role::BallDefender>("defender_6"),
        std::make_unique<role::Formation>("attacker_1"),
    };
}

uint8_t DefendPass::score(const rtt::Field& field) noexcept {
    auto enemyRobot = world->getWorld()->getRobotClosestToBall(world::them);
    auto position = distanceFromPointToLine(field.playArea.bottomLeft(), field.playArea.topLeft(), enemyRobot->get()->getPos());
    auto goalVisibility =
        FieldComputations::getPercentageOfGoalVisibleFromPoint(field, true, enemyRobot->get()->getPos(), world->getWorld().value(), enemyRobot->get()->getId(), false);
    return 255 * (position / field.playArea.width()) * (100 - goalVisibility) / 100;
}

Dealer::FlagMap DefendPass::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;

    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER);

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {keeperFlag}}});
    flagMap.insert({"harasser", {DealerFlagPriority::REQUIRED, {}}});
    flagMap.insert({"defender_0", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"defender_1", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"defender_2", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"defender_3", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"defender_4", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"defender_5", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"defender_6", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"attacker_0", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"attacker_1", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});

    return flagMap;
}

void DefendPass::calculateInfoForRoles() noexcept {
    PositionComputations::calculateInfoForKeeper(stpInfos, field, world);
    PositionComputations::calculateInfoForHarasser(stpInfos, &roles, field, world);
    PositionComputations::calculateInfoForDefenders(stpInfos, roles, field, world);
    PositionComputations::calculateInfoForAttackers(stpInfos, roles, field, world);
}

const char* DefendPass::getName() const { return "Defend Pass"; }

}  // namespace rtt::ai::stp::play
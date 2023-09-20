//
// Created by Luuk and Jorn on 12/09/2023.
//

#include "stp/plays/defensive/KeeperDemo.h"

#include "stp/computations/PositionScoring.h"
#include "stp/roles/DemoKeeper.h"
#include "stp/constants/ControlConstants.h"
#include "stp/roles/passive/Halt.h"


namespace rtt::ai::stp::play {

KeeperDemo::KeeperDemo() : Play() {
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(eval::NormalPlayGameState);

    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(eval::NormalPlayGameState);

    roles = std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT>{std::make_unique<role::DemoKeeper>(("keeper")),                                                                                       std::make_unique<role::Halt>(role::Halt("halt_0")),
                                                                                       std::make_unique<role::Halt>(role::Halt("halt_1")),
                                                                                       std::make_unique<role::Halt>(role::Halt("halt_2")),
                                                                                       std::make_unique<role::Halt>(role::Halt("halt_3")),
                                                                                       std::make_unique<role::Halt>(role::Halt("halt_4")),
                                                                                       std::make_unique<role::Halt>(role::Halt("halt_5")),
                                                                                       std::make_unique<role::Halt>(role::Halt("halt_6")),
                                                                                       std::make_unique<role::Halt>(role::Halt("halt_7")),
                                                                                       std::make_unique<role::Halt>(role::Halt("halt_8")),
                                                                                       std::make_unique<role::Halt>(role::Halt("halt_9"))};
}

uint8_t KeeperDemo::score(const rtt::Field& field) noexcept {
    // never play automatically, only when the user selects it
    return control_constants::FUZZY_FALSE;
}

Dealer::FlagMap KeeperDemo::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {}}});
    flagMap.insert({"halt_0", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_1", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_2", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_3", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_4", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_5", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_6", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_7", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_8", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_9", {DealerFlagPriority::LOW_PRIORITY, {}}});


    return flagMap;
}

void KeeperDemo::calculateInfoForRoles() noexcept {
    stpInfos["keeper"].setPositionToMoveTo(field.leftGoalArea.rightLine().center());
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));
}

const char* KeeperDemo::getName() const { return "Keeper Demo"; }

}  // namespace rtt::ai::stp::play

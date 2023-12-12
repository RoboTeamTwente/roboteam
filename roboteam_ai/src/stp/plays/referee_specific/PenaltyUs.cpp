//
// Created by timovdk on 5/1/20.
//

#include "stp/plays/referee_specific/PenaltyUs.h"

#include "stp/computations/GoalComputations.h"
#include "stp/computations/PositionComputations.h"
#include "stp/roles/Keeper.h"
#include "stp/roles/active/PenaltyTaker.h"
#include "stp/roles/passive/Halt.h"

namespace rtt::ai::stp::play {
const char* PenaltyUs::getName() const { return "Penalty Us"; }

PenaltyUs::PenaltyUs() : Play() {
    // Evaluations that have to be true in order for this play to be considered valid.
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(GlobalEvaluation::PenaltyUsGameState);

    // Evaluations that have to be true to allow the play to continue, otherwise the play will change. Plays can also end using the shouldEndPlay().
    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(GlobalEvaluation::PenaltyUsGameState);

    // Role creation, the names should be unique. The names are used in the stpInfos-map.
    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
        // Roles is we play 6v6
        std::make_unique<role::Keeper>("keeper"), std::make_unique<role::PenaltyTaker>("kicker"), std::make_unique<role::Halt>("halt_0"), std::make_unique<role::Halt>("halt_1"),
        std::make_unique<role::Halt>("halt_2"), std::make_unique<role::Halt>("halt_3"),
        // Additional roles if we play 11v11
        std::make_unique<role::Halt>("halt_4"), std::make_unique<role::Halt>("halt_5"), std::make_unique<role::Halt>("halt_6"), std::make_unique<role::Halt>("halt_7"),
        std::make_unique<role::Halt>("halt_8")};
}

uint8_t PenaltyUs::score(const rtt::Field& field) noexcept {
    // If this play is valid we always want to execute this play
    return control_constants::FUZZY_TRUE;
}

Dealer::FlagMap PenaltyUs::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;

    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER);
    Dealer::DealerFlag kickerFlag(DealerFlagTitle::CAN_KICK_BALL);
    Dealer::DealerFlag detectionFlag(DealerFlagTitle::CAN_DETECT_BALL);

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {keeperFlag}}});
    flagMap.insert({"kicker", {DealerFlagPriority::REQUIRED, {kickerFlag, detectionFlag}}});
    flagMap.insert({"halt_0", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_1", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_2", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_3", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_4", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_5", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_6", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_7", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_8", {DealerFlagPriority::LOW_PRIORITY, {}}});

    return flagMap;
}

void PenaltyUs::calculateInfoForRoles() noexcept {
    // Function where are roles get their information, make sure not to compute roles twice.
    PositionComputations::calculateInfoForKeeper(stpInfos, field, world);

    auto positionTarget = PositionComputations::getPosition(std::nullopt, field.middleRightGrid, gen::OffensivePosition, field, world);
    stpInfos["kicker"].setPositionToMoveTo(positionTarget);
    auto goalTarget = computations::GoalComputations::calculateGoalTarget(world, field);
    stpInfos["kicker"].setPositionToShootAt(goalTarget);
    stpInfos["kicker"].setShotType(ShotType::MAX);
    if (stpInfos["kicker"].getRobot().has_value())
        stpInfos["kicker"].setMaxRobotVelocity(std::max((stpInfos["kicker"].getRobot()->get()->getPos() - positionTarget.position).length(), 0.5));
}

}  // namespace rtt::ai::stp::play

#include "stp/plays/referee_specific/PenaltyThem.h"

#include "stp/roles/Keeper.h"
#include "stp/roles/PenaltyKeeper.h"
#include "stp/roles/passive/Halt.h"

namespace rtt::ai::stp::play {

PenaltyThem::PenaltyThem() : Play() {
    // Evaluations that have to be true in order for this play to be considered valid.
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(GlobalEvaluation::PenaltyThemGameState);

    // Evaluations that have to be true to allow the play to continue, otherwise the play will change. Plays can also end using the shouldEndPlay().
    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(GlobalEvaluation::PenaltyThemGameState);

    // Role creation, the names should be unique. The names are used in the stpInfos-map.
    roles = std::array<std::unique_ptr<Role>, rtt::ai::constants::MAX_ROBOT_COUNT>{
        // Roles is we play 6v6
        std::make_unique<role::PenaltyKeeper>("keeper"), std::make_unique<role::Halt>("halt_0"), std::make_unique<role::Halt>("halt_1"), std::make_unique<role::Halt>("halt_2"),
        std::make_unique<role::Halt>("halt_3"), std::make_unique<role::Halt>("halt_4"),
        // Additional roles if we play 11v11
        std::make_unique<role::Halt>("halt_5"), std::make_unique<role::Halt>("halt_6"), std::make_unique<role::Halt>("halt_7"), std::make_unique<role::Halt>("halt_8"),
        std::make_unique<role::Halt>("halt_9")};
}

uint8_t PenaltyThem::score(const rtt::Field&) noexcept {
    // If this play is valid we always want to execute this play
    return constants::FUZZY_TRUE;
}

Dealer::FlagMap PenaltyThem::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER);

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {keeperFlag}}});
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

void PenaltyThem::calculateInfoForRoles() noexcept {
    if (stpInfos["keeper"].getRobot() && field.leftDefenseArea.rightLine().distanceToLine(world->getWorld()->getBall()->get()->position) < 0.8 &&
        field.leftDefenseArea.rightLine().center().x < world->getWorld()->getBall()->get()->position.x &&
        world->getWorld()->getBall()->get()->velocity.length() < constants::BALL_GOT_SHOT_LIMIT) {
        // If the ball gets close to the goal, we want to get closer towards the ball to make it harder to score
        auto targetPosition = world->getWorld()->getBall()->get()->position -
                              (world->getWorld()->getBall()->get()->position - field.leftGoalArea.rightLine().center()).stretchToLength(constants::ROBOT_RADIUS * 2.5);
        stpInfos["keeper"].setPositionToMoveTo(targetPosition);
    } else
        // Just stand in the middle of the goal
        stpInfos["keeper"].setPositionToMoveTo(field.leftGoalArea.rightLine().center());
}

const char* PenaltyThem::getName() const { return "Penalty Them"; }
}  // namespace rtt::ai::stp::play
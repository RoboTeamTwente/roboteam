//
// Created by jordi on 30-04-20.
//

#include "stp/plays/referee_specific/KickOffThemPrepare.h"

#include "stp/roles/passive/Formation.h"

namespace rtt::ai::stp::play {

KickOffThemPrepare::KickOffThemPrepare() : Play() {
    /// Evaluations that have to be true to be considered when changing plays.
    startPlayEvaluation.clear();  // DONT TOUCH.
    startPlayEvaluation.emplace_back(eval::KickOffThemPrepareGameState);

    /// Evaluations that have to be true to allow the play to continue, otherwise the play will change. Plays can also end using the shouldEndPlay().
    keepPlayEvaluation.clear();  // DONT TOUCH.
    keepPlayEvaluation.emplace_back(eval::KickOffThemPrepareGameState);

    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
        std::make_unique<role::Formation>("keeper"),
        std::make_unique<role::Formation>("formation_0"),
        std::make_unique<role::Formation>("formation_1"),
        std::make_unique<role::Formation>("formation_2"),
        std::make_unique<role::Formation>("formation_3"),
        std::make_unique<role::Formation>("formation_4"),
        std::make_unique<role::Formation>("formation_5"),
        std::make_unique<role::Formation>("formation_6"),
        std::make_unique<role::Formation>("formation_7"),
        std::make_unique<role::Formation>("formation_8"),
        std::make_unique<role::Formation>("formation_9")
    };
}

uint8_t KickOffThemPrepare::score(const rtt::Field& field) noexcept {
    /// List of all factors that combined results in an evaluation how good the play is.
    scoring = {{PlayEvaluator::getGlobalEvaluation(eval::KickOffThemPrepareGameState, world), 1.0}};
    return (lastScore = PlayEvaluator::calculateScore(scoring)).value();  // DONT TOUCH.
}

void KickOffThemPrepare::calculateInfoForRoles() noexcept {
    auto height = field.playArea.height();
    auto width = field.playArea.width();

    // Keeper
    stpInfos["keeper"].setPositionToMoveTo(Vector2(field.leftGoalArea.rightLine().center() + Vector2(0.5, 0.0)));
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));

    double defense_line_x = field.leftDefenseArea.right() + control_constants::DEFENSE_AREA_AVOIDANCE_MARGIN;
    // regular bots
    stpInfos["formation_0"].setPositionToMoveTo(Vector2(-width / 4, height / 8));
    stpInfos["formation_1"].setPositionToMoveTo(Vector2(-width / 4, -height / 8));
    stpInfos["formation_2"].setPositionToMoveTo(Vector2(-width / 8, height / 4));
    stpInfos["formation_3"].setPositionToMoveTo(Vector2(-width / 8, -height / 4));
    stpInfos["formation_4"].setPositionToMoveTo(Vector2(defense_line_x, 0.0));
    stpInfos["formation_5"].setPositionToMoveTo(Vector2(defense_line_x, height / 5));
    stpInfos["formation_6"].setPositionToMoveTo(Vector2(defense_line_x, -height / 5));
    stpInfos["formation_7"].setPositionToMoveTo(Vector2(-width / 4, height / 3));
    stpInfos["formation_8"].setPositionToMoveTo(Vector2(-width / 4, -height / 3));
    stpInfos["formation_9"].setPositionToMoveTo(Vector2(-width * 3.0 / 16.0, 0.0));
}

Dealer::FlagMap KickOffThemPrepare::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {}}});
    flagMap.insert({"formation_0", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_1", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_2", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_3", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_4", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_5", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_6", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_7", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_8", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_9", {DealerFlagPriority::LOW_PRIORITY, {}}});

    return flagMap;
}

const char* KickOffThemPrepare::getName() const { return "Kick Off Them Prepare"; }

}  // namespace rtt::ai::stp::play
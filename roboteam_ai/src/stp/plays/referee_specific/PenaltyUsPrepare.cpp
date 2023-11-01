#include "stp/plays/referee_specific/PenaltyUsPrepare.h"

#include "stp/roles/passive/BallAvoider.h"
#include "stp/roles/passive/Formation.h"

namespace rtt::ai::stp::play {

PenaltyUsPrepare::PenaltyUsPrepare() : Play() {
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(eval::PenaltyUsPrepareGameState);

    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(eval::PenaltyUsPrepareGameState);

    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
        // Roles is we play 6v6
        std::make_unique<role::Formation>("keeper"),
        std::make_unique<role::BallAvoider>("kicker_formation"),
        std::make_unique<role::BallAvoider>("formation_0"),
        std::make_unique<role::BallAvoider>("formation_1"),
        std::make_unique<role::BallAvoider>("formation_2"),
        std::make_unique<role::BallAvoider>("formation_3"),
        // Additional roles if we play 11v11
        std::make_unique<role::BallAvoider>("formation_4"),
        std::make_unique<role::BallAvoider>("formation_5"),
        std::make_unique<role::BallAvoider>("formation_6"),
        std::make_unique<role::BallAvoider>("formation_7"),
        std::make_unique<role::BallAvoider>("formation_8"),
    };
}

uint8_t PenaltyUsPrepare::score(const rtt::Field& field) noexcept {
    /// List of all factors that combined results in an evaluation how good the play is.
    scoring = {{PlayEvaluator::getGlobalEvaluation(eval::PenaltyUsPrepareGameState, world), 1.0}};
    return (lastScore = PlayEvaluator::calculateScore(scoring)).value();  // DONT TOUCH.
}

Dealer::FlagMap PenaltyUsPrepare::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;

    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER);
    Dealer::DealerFlag kickerFlag(DealerFlagTitle::CAN_KICK_BALL);
    Dealer::DealerFlag detectionFlag(DealerFlagTitle::CAN_DETECT_BALL);

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {keeperFlag}}});
    flagMap.insert({"kicker_formation", {DealerFlagPriority::REQUIRED, {kickerFlag, detectionFlag}}});
    flagMap.insert({"formation_0", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_1", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_2", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_3", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_4", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_5", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_6", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_7", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_8", {DealerFlagPriority::LOW_PRIORITY, {}}});

    return flagMap;
}

void PenaltyUsPrepare::calculateInfoForRoles() noexcept {
    // We need at least a keeper, and a kicker positioned behind the ball
    PositionComputations::calculateInfoForPenalty(stpInfos, field, world);
    stpInfos["keeper"].setPositionToMoveTo(Vector2(field.leftGoalArea.rightLine().center()));
    stpInfos["kicker_formation"].setPositionToMoveTo(world->getWorld()->getBall()->get()->position - Vector2{0.25, 0.0});
}

const char* PenaltyUsPrepare::getName() const { return "Penalty Us Prepare"; }
}  // namespace rtt::ai::stp::play

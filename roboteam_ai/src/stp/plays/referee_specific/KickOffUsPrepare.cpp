//
// Created by jordi on 30-04-20.
//

#include "stp/plays/referee_specific/KickOffUsPrepare.h"

#include "stp/roles/Keeper.h"
#include "stp/roles/passive/Formation.h"

namespace rtt::ai::stp::play {

KickOffUsPrepare::KickOffUsPrepare() : Play() {
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(eval::KickOffUsPrepareGameState);

    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(eval::KickOffUsPrepareGameState);

    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
        // Roles is we play 6v6
        std::make_unique<role::Formation>("keeper"),
        std::make_unique<role::Formation>("kicker"),
        std::make_unique<role::Formation>("formation_mid_0"),
        std::make_unique<role::Formation>("formation_back_0"),
        std::make_unique<role::Formation>("formation_front_0"),
        std::make_unique<role::Formation>("formation_mid_1"),
        // Additional roles if we play 11v11
        std::make_unique<role::Formation>("formation_front_1"),
        std::make_unique<role::Formation>("formation_back_1"),
        std::make_unique<role::Formation>("formation_mid_2"),
        std::make_unique<role::Formation>("formation_front_2"),
        std::make_unique<role::Formation>("formation_back_2"),

    };
}

uint8_t KickOffUsPrepare::score(const rtt::Field& field) noexcept {
    /// List of all factors that combined results in an evaluation how good the play is.
    scoring = {{PlayEvaluator::getGlobalEvaluation(eval::KickOffUsPrepareGameState, world), 1.0}};
    return (lastScore = PlayEvaluator::calculateScore(scoring)).value();
}

Dealer::FlagMap KickOffUsPrepare::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER);

    Dealer::DealerFlag kickerFlag(DealerFlagTitle::CAN_KICK_BALL);
    Dealer::DealerFlag detectionFlag(DealerFlagTitle::CAN_DETECT_BALL);

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {keeperFlag}}});
    flagMap.insert({"kicker", {DealerFlagPriority::REQUIRED, {kickerFlag, detectionFlag}}});
    flagMap.insert({"formation_back_0", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"formation_back_1", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"formation_back_2", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"formation_mid_0", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_mid_1", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_mid_2", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_front_0", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"formation_front_1", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"formation_front_2", {DealerFlagPriority::HIGH_PRIORITY, {}}});

    return flagMap;
}

void KickOffUsPrepare::calculateInfoForRoles() noexcept {
    PositionComputations::calculateInfoForKeeper(stpInfos, field, world);
    PositionComputations::calculateInfoForFormationOurSide(stpInfos, roles, field, world);

    // The "kicker" will go to the ball
    if (stpInfos["kicker"].getRobot() && stpInfos["kicker"].getRobot()->get()->getPos().x < 0) {
        Vector2 robotPos = stpInfos["kicker"].getRobot()->get()->getPos();
        Vector2 ballPos = world->getWorld()->getBall()->get()->position;
        if ((robotPos - ballPos).length() < 0.7) {
            stpInfos["kicker"].setPositionToMoveTo(Vector2(-0.25, 0.0));
        } else if (robotPos.y > 0) {
            stpInfos["kicker"].setPositionToMoveTo(Vector2(-0.25, 0.6));
        } else {
            stpInfos["kicker"].setPositionToMoveTo(Vector2(-0.25, -0.6));
        }
    } else {
        stpInfos["kicker"].setPositionToMoveTo(Vector2(-0.25, 0.0));
    }
}

const char* KickOffUsPrepare::getName() const { return "Kick Off Us Prepare"; }

}  // namespace rtt::ai::stp::play
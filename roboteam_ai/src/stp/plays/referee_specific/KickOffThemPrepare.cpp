#include "stp/plays/referee_specific/KickOffThemPrepare.h"

#include "stp/roles/Keeper.h"
#include "stp/roles/passive/Formation.h"

namespace rtt::ai::stp::play {

KickOffThemPrepare::KickOffThemPrepare() : Play() {
    // Evaluations that have to be true in order for this play to be considered valid.
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(GlobalEvaluation::KickOffThemPrepareGameState);

    // Evaluations that have to be true to allow the play to continue, otherwise the play will change. Plays can also end using the shouldEndPlay().
    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(GlobalEvaluation::KickOffThemPrepareGameState);

    // Role creation, the names should be unique. The names are used in the stpInfos-map.
    roles = std::array<std::unique_ptr<Role>, rtt::ai::constants::MAX_ROBOT_COUNT>{
        // Roles is we play 6v6
        std::make_unique<role::Keeper>("keeper"),
        std::make_unique<role::Formation>("kicker"),
        std::make_unique<role::Formation>("formation_front_0"),
        std::make_unique<role::Formation>("formation_front_3"),
        std::make_unique<role::Formation>("winger_1"),
        std::make_unique<role::Formation>("winger_2"),
        // Roles is we play 11v11
        std::make_unique<role::Formation>("formation_front_1"),
        std::make_unique<role::Formation>("formation_front_2"),
        std::make_unique<role::Formation>("centerback"),
        std::make_unique<role::Formation>("rightback"),
        std::make_unique<role::Formation>("leftback"),
    };
}

uint8_t KickOffThemPrepare::score(const rtt::Field&) noexcept {
    // If this play is valid we always want to execute this play
    return constants::FUZZY_TRUE;
}

Dealer::FlagMap KickOffThemPrepare::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER);

    Dealer::DealerFlag kickerFlag(DealerFlagTitle::CAN_KICK_BALL);
    Dealer::DealerFlag detectionFlag(DealerFlagTitle::CAN_DETECT_BALL);

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {keeperFlag}}});
    flagMap.insert({"kicker", {DealerFlagPriority::REQUIRED, {kickerFlag, detectionFlag}}});
    flagMap.insert({"formation_front_0", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"formation_front_1", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"formation_front_2", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"formation_front_3", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"leftback", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"rightback", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"centerback", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"winger_1", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"winger_2", {DealerFlagPriority::LOW_PRIORITY, {}}});

    return flagMap;
}

void KickOffThemPrepare::calculateInfoForRoles() noexcept {
    PositionComputations::calculateInfoForDefendersAndWallers(stpInfos, roles, field, world, true);
    PositionComputations::calculateInfoForFormationOurSide(stpInfos, roles, field, world);

    // The "kicker" will go to the ball
    stpInfos["kicker"].setPositionToMoveTo(Vector2(-constants::AVOID_BALL_DISTANCE, -constants::AVOID_BALL_DISTANCE));
    
    // Assign positions to roles
    stpInfos["formation_front_0"].setPositionToMoveTo(Vector2(-0.5, 3.0));
    stpInfos["formation_front_1"].setPositionToMoveTo(Vector2(-0.5, 2.0));
    stpInfos["formation_front_2"].setPositionToMoveTo(Vector2(-0.5, -2.0));
    stpInfos["formation_front_3"].setPositionToMoveTo(Vector2(-0.5, -3.0));

    stpInfos["leftback"].setPositionToMoveTo(Vector2(-3.0, 1.5));
    stpInfos["rightback"].setPositionToMoveTo(Vector2(-3.0, -1.5));
    stpInfos["centerback"].setPositionToMoveTo(Vector2(-4.2, 0));

    stpInfos["winger_1"].setPositionToMoveTo(Vector2(-2.3, 3.5));
    stpInfos["winger_2"].setPositionToMoveTo(Vector2(-2.3, -3.5));
}
const char* KickOffThemPrepare::getName() const { return "Kick Off Them Prepare"; }

}  // namespace rtt::ai::stp::play
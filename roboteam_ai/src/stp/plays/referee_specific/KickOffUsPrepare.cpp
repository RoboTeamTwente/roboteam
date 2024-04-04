#include "stp/plays/referee_specific/KickOffUsPrepare.h"

#include "stp/roles/Keeper.h"
#include "stp/roles/passive/Formation.h"

namespace rtt::ai::stp::play {

KickOffUsPrepare::KickOffUsPrepare() : Play() {
    // Evaluations that have to be true in order for this play to be considered valid.
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(GlobalEvaluation::KickOffUsPrepareGameState);

    // Evaluations that have to be true to allow the play to continue, otherwise the play will change. Plays can also end using the shouldEndPlay().
    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(GlobalEvaluation::KickOffUsPrepareGameState);

    // Role creation, the names should be unique. The names are used in the stpInfos-map.
    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
        // Roles is we play 6v6
        std::make_unique<role::Keeper>("keeper"),
        std::make_unique<role::Formation>("kicker"),
        std::make_unique<role::Formation>("formation_mid_0"),
        std::make_unique<role::Formation>("formation_front_0"),
        std::make_unique<role::Formation>("formation_front_1"),
        std::make_unique<role::Formation>("formation_front_2"),
        // Additional roles if we play 11v11
        std::make_unique<role::Formation>("formation_back_0"),
        std::make_unique<role::Formation>("formation_back_1"),
        std::make_unique<role::Formation>("formation_front_3"),
        std::make_unique<role::Formation>("formation_front_4"),
        std::make_unique<role::Formation>("formation_front_5"),
    };
}

uint8_t KickOffUsPrepare::score(const rtt::Field&) noexcept {
    // If this play is valid we always want to execute this play
    return control_constants::FUZZY_TRUE;
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
    flagMap.insert({"formation_mid_0", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_front_0", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"formation_front_1", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"formation_front_2", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"formation_front_3", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"formation_front_4", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"formation_front_5", {DealerFlagPriority::HIGH_PRIORITY, {}}});

    return flagMap;
}

void KickOffUsPrepare::calculateInfoForRoles() noexcept {
    PositionComputations::calculateInfoForFormationOurSide(stpInfos, roles, field, world);

    // The "kicker" will go to the ball
    stpInfos["kicker"].setPositionToMoveTo(Vector2(-control_constants::AVOID_BALL_DISTANCE, 0.0));
}

const char* KickOffUsPrepare::getName() const { return "Kick Off Us Prepare"; }

}  // namespace rtt::ai::stp::play
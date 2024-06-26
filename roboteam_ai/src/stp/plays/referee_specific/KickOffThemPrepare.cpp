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
        std::make_unique<role::Formation>("formation_back_0"),
        std::make_unique<role::Formation>("formation_mid_0"),
        std::make_unique<role::Formation>("formation_front_0"),
        std::make_unique<role::Formation>("formation_back_1"),
        std::make_unique<role::Formation>("formation_mid_1"),
        // Additional roles if we play 11v11
        std::make_unique<role::Formation>("formation_front_1"),
        std::make_unique<role::Formation>("formation_back_2"),
        std::make_unique<role::Formation>("formation_mid_2"),
        std::make_unique<role::Formation>("formation_front_2"),
        std::make_unique<role::Formation>("formation_mid_3"),
    };
}

uint8_t KickOffThemPrepare::score(const rtt::Field&) noexcept {
    // If this play is valid we always want to execute this play
    return constants::FUZZY_TRUE;
}

Dealer::FlagMap KickOffThemPrepare::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER);

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {keeperFlag}}});
    flagMap.insert({"formation_back_0", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"formation_back_1", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"formation_back_2", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"formation_mid_0", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_mid_1", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_mid_2", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_mid_3", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_front_0", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"formation_front_1", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"formation_front_2", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});

    return flagMap;
}

void KickOffThemPrepare::calculateInfoForRoles() noexcept {
    PositionComputations::calculateInfoForFormationOurSide(stpInfos, roles, field, world);
    stpInfos["formation_front_0"].setPositionToMoveTo(Vector2(-0.73, 0));
}
const char* KickOffThemPrepare::getName() const { return "Kick Off Them Prepare"; }

}  // namespace rtt::ai::stp::play
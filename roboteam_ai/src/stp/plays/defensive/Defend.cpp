#include "stp/plays/defensive/Defend.h"

#include <stp/roles/passive/Formation.h>

#include "stp/roles/Keeper.h"
#include "stp/roles/active/Harasser.h"
#include "stp/roles/passive/Defender.h"

namespace rtt::ai::stp::play {

Defend::Defend() : Play() {
    // Evaluations that have to be true in order for this play to be considered valid.
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(GlobalEvaluation::NormalPlayGameState);
    startPlayEvaluation.emplace_back(GlobalEvaluation::WeWillNotHaveBall);

    // Evaluations that have to be true to allow the play to continue, otherwise the play will change. Plays can also end using the shouldEndPlay().
    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(GlobalEvaluation::NormalPlayGameState);
    keepPlayEvaluation.emplace_back(GlobalEvaluation::TheyHaveBall);

    // Role creation, the names should be unique. The names are used in the stpInfos-map.
    roles = std::array<std::unique_ptr<Role>, rtt::ai::constants::MAX_ROBOT_COUNT>{
        // Roles is we play 6v6
        std::make_unique<role::Harasser>("harasser"),
        std::make_unique<role::Defender>("defender_0"),
        std::make_unique<role::Defender>("defender_1"),
        std::make_unique<role::Defender>("defender_2"),
        std::make_unique<role::Defender>("defender_3"),
        std::make_unique<role::Defender>("defender_4"),
        std::make_unique<role::Defender>("defender_5"),
        std::make_unique<role::Defender>("defender_6"),
        std::make_unique<role::Defender>("defender_7"),
        std::make_unique<role::Defender>("defender_8"),
        std::make_unique<role::Defender>("defender_9"),

    };
}

uint8_t Defend::score(const rtt::Field&) noexcept {
    // If this play is valid we always want to execute this play
    return constants::FUZZY_TRUE;
}

Dealer::FlagMap Defend::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;

    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER);

    flagMap.insert({"harasser", {DealerFlagPriority::REQUIRED, {}, harasserInfo.interceptId}});
    flagMap.insert({"defender_0", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"defender_1", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"defender_2", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"defender_3", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"defender_4", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"defender_5", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"defender_6", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"defender_7", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"defender_8", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"defender_9", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});

    return flagMap;
}

void Defend::calculateInfoForRoles() noexcept {
    harasserInfo = InterceptionComputations::calculateInterceptionInfoExcludingKeeperAndCarded(world);
    PositionComputations::calculateInfoForHarasser(stpInfos, &roles, field, world, harasserInfo.interceptLocation);
    PositionComputations::calculateInfoForDefendersAndWallers(stpInfos, roles, field, world, false);
}

const char* Defend::getName() const { return "Defend"; }

}  // namespace rtt::ai::stp::play

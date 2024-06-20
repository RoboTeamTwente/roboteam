#include "stp/plays/referee_specific/StopFormation.h"

#include "stp/roles/Keeper.h"
#include "stp/roles/active/Harasser.h"
#include "stp/roles/passive/Defender.h"
#include "stp/roles/passive/Formation.h"

namespace rtt::ai::stp::play {

StopFormation::StopFormation() : Play() {
    // Evaluations that have to be true in order for this play to be considered valid.
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(GlobalEvaluation::StopGameState);

    // Evaluations that have to be true to allow the play to continue, otherwise the play will change. Plays can also end using the shouldEndPlay().
    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(GlobalEvaluation::StopGameState);

    // Role creation, the names should be unique. The names are used in the stpInfos-map.
    roles = std::array<std::unique_ptr<Role>, rtt::ai::constants::MAX_ROBOT_COUNT>{
        // Roles is we play 6v6
        std::make_unique<role::Keeper>("keeper"),
        std::make_unique<role::Formation>("harasser"),
        std::make_unique<role::Defender>("waller_0"),
        std::make_unique<role::Defender>("waller_1"),
        std::make_unique<role::Defender>("defender_0"),
        std::make_unique<role::Defender>("defender_1"),
        // Additional roles if we play 11v11
        std::make_unique<role::Defender>("defender_2"),
        std::make_unique<role::Defender>("waller_2"),
        std::make_unique<role::Formation>("attacker_0"),
        std::make_unique<role::Defender>("waller_3"),
        std::make_unique<role::Defender>("defender_3"),
    };
}

uint8_t StopFormation::score(const rtt::Field&) noexcept {
    // If this play is valid we always want to execute this play
    return constants::FUZZY_TRUE;
}

Dealer::FlagMap StopFormation::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER);

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {keeperFlag}}});
    flagMap.insert({"harasser", {DealerFlagPriority::REQUIRED, {}}});
    for (int i = 0; i < Play::waller_count; i++) {
        if (i <= PositionComputations::amountOfWallers) {
            flagMap.insert({"waller_" + std::to_string(i), {DealerFlagPriority::HIGH_PRIORITY}});
        } else
            flagMap.insert({"waller_" + std::to_string(i), {DealerFlagPriority::MEDIUM_PRIORITY}});
    }
    flagMap.insert({"defender_0", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"defender_1", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"defender_2", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"defender_3", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"attacker_0", {DealerFlagPriority::LOW_PRIORITY, {}}});

    return flagMap;
}

void StopFormation::calculateInfoForRoles() noexcept {
    PositionComputations::calculateInfoForDefendersAndWallers(stpInfos, roles, field, world, false);
    PositionComputations::calculateInfoForAttackers(stpInfos, roles, field, world);
    PositionComputations::calculateInfoForAvoidBallHarasser(stpInfos, world);
}

const char* StopFormation::getName() const { return "Stop Formation"; }
}  // namespace rtt::ai::stp::play
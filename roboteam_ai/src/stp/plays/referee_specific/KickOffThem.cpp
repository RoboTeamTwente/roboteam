#include "stp/plays/referee_specific/KickOffThem.h"

#include "stp/roles/Keeper.h"
#include "stp/roles/passive/Halt.h"
#include "utilities/GameStateManager.hpp"

namespace rtt::ai::stp::play {

KickOffThem::KickOffThem() : Play() {
    // Evaluations that have to be true in order for this play to be considered valid.
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(GlobalEvaluation::KickOffThemGameState);

    // Evaluations that have to be true to allow the play to continue, otherwise the play will change. Plays can also end using the shouldEndPlay().
    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(GlobalEvaluation::KickOffThemGameState);

    // Role creation, the names should be unique. The names are used in the stpInfos-map.
    roles = std::array<std::unique_ptr<Role>, rtt::ai::constants::MAX_ROBOT_COUNT>{
        // Roles is we play 6v6
        std::make_unique<role::Keeper>("keeper"),
        std::make_unique<role::Halt>("halt_0"),
        std::make_unique<role::Halt>("halt_1"),
        std::make_unique<role::Halt>("halt_2"),
        std::make_unique<role::Halt>("halt_3"),
        std::make_unique<role::Halt>("halt_4"),
        // Additional roles if we play 11v11
        std::make_unique<role::Halt>("halt_5"),
        std::make_unique<role::Halt>("halt_6"),
        std::make_unique<role::Halt>("halt_7"),
        std::make_unique<role::Halt>("halt_8"),
        std::make_unique<role::Halt>("halt_9"),
    };
}

uint8_t KickOffThem::score(const rtt::Field&) noexcept {
    // If this play is valid we always want to execute this play
    return constants::FUZZY_TRUE;
}

Dealer::FlagMap KickOffThem::decideRoleFlags() const noexcept {
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

void KickOffThem::calculateInfoForRoles() noexcept {
    Vector2 receiver = Vector2(-2.3, 3.5);
    stpInfos["kick_off_taker"].setPositionToShootAt(receiver);
    stpInfos["kick_off_taker"].setShotPower(ShotPower::KICKOFF);
    stpInfos["kick_off_taker"].setKickOrChip(KickType::KICK);
    stpInfos["kick_off_taker"].setShootOnFirstTouch(true);
    stpInfos["kick_off_taker"].setShouldAvoidOurRobots(false);

}

bool KickOffThem::shouldEndPlay() noexcept { return false; }

const char* KickOffThem::getName() const { return "Kick Off Them"; }

}  // namespace rtt::ai::stp::play

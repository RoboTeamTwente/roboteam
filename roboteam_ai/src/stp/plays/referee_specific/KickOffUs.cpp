#include "stp/plays/referee_specific/KickOffUs.h"

#include "stp/roles/Keeper.h"
#include "stp/roles/active/FreeKickTaker.h"
#include "stp/roles/passive/Halt.h"
#include "utilities/GameStateManager.hpp"

namespace rtt::ai::stp::play {

KickOffUs::KickOffUs() : Play() {
    // Evaluations that have to be true in order for this play to be considered valid.
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(GlobalEvaluation::KickOffUsGameState);

    // Evaluations that have to be true to allow the play to continue, otherwise the play will change. Plays can also end using the shouldEndPlay().
    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(GlobalEvaluation::KickOffUsGameState);

    // Role creation, the names should be unique. The names are used in the stpInfos-map.
    roles = std::array<std::unique_ptr<Role>, rtt::ai::constants::MAX_ROBOT_COUNT>{
        // Roles is we play 6v6
        std::make_unique<role::Keeper>("keeper"),
        std::make_unique<role::FreeKickTaker>("kick_off_taker"),
        std::make_unique<role::Halt>("halt_0"),
        std::make_unique<role::Halt>("halt_1"),
        std::make_unique<role::Halt>("halt_2"),
        std::make_unique<role::Halt>("halt_3"),
        // Additional roles if we play 11v11
        std::make_unique<role::Halt>("halt_4"),
        std::make_unique<role::Halt>("halt_5"),
        std::make_unique<role::Halt>("halt_6"),
        std::make_unique<role::Halt>("halt_7"),
        std::make_unique<role::Halt>("halt_8"),
    };
}

uint8_t KickOffUs::score(const rtt::Field &) noexcept {
    // If this play is valid we always want to execute this play
    return constants::FUZZY_TRUE;
}

Dealer::FlagMap KickOffUs::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;

    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER);

    Dealer::DealerFlag kickerFlag(DealerFlagTitle::CAN_KICK_BALL);
    Dealer::DealerFlag detectionFlag(DealerFlagTitle::CAN_DETECT_BALL);

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {keeperFlag}}});
    flagMap.insert({"kick_off_taker", {DealerFlagPriority::REQUIRED, {kickerFlag, detectionFlag}}});
    flagMap.insert({"halt_0", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_1", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_2", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_3", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_4", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_5", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_6", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_7", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_8", {DealerFlagPriority::LOW_PRIORITY, {}}});

    return flagMap;
}

void KickOffUs::calculateInfoForRoles() noexcept {
    Vector2 theirGoal = Vector2(6, 0);
    stpInfos["kick_off_taker"].setPositionToShootAt(theirGoal);
    stpInfos["kick_off_taker"].setShotPower(ShotPower::PASS);
    stpInfos["kick_off_taker"].setKickOrChip(KickType::CHIP);
    stpInfos["kick_off_taker"].setShootOnFirstTouch(true);
}

bool KickOffUs::shouldEndPlay() noexcept {
    // If the ball is kicked, we end the play to already prepare for what happens next
    if (ballKicked()) return true;
    return false;
}

bool KickOffUs::ballKicked() {
    return std::any_of(roles.begin(), roles.end(), [](const std::unique_ptr<Role> &role) {
        return role != nullptr && role->getName() == "kick_off_taker" && strcmp(role->getCurrentTactic()->getName(), "Formation") == 0;
    });
}

const char *KickOffUs::getName() const { return "Kick Off Us"; }
}  // namespace rtt::ai::stp::play

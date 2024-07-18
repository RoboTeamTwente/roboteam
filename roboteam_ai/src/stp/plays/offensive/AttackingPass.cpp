#include "stp/plays/offensive/AttackingPass.h"

#include <roboteam_utils/Hungarian.h>
#include <roboteam_utils/LineSegment.h>

#include "stp/computations/PassComputations.h"
#include "stp/computations/PositionScoring.h"
#include "stp/roles/Keeper.h"
#include "stp/roles/active/PassReceiver.h"
#include "stp/roles/active/Passer.h"
#include "stp/roles/passive/Defender.h"
#include "stp/roles/passive/Formation.h"
#include "utilities/Constants.h"
#include "world/views/RobotView.hpp"

namespace rtt::ai::stp::play {
AttackingPass::AttackingPass() : Play() {
    // Evaluations that have to be true in order for this play to be considered valid.
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(GlobalEvaluation::NormalPlayGameState);
    startPlayEvaluation.emplace_back(GlobalEvaluation::WeWillHaveBall);

    // Evaluations that have to be true to allow the play to continue, otherwise the play will change. Plays can also end using the shouldEndPlay().
    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(GlobalEvaluation::NormalPlayGameState);
    keepPlayEvaluation.emplace_back(GlobalEvaluation::WeWillHaveBall);

    // Role creation, the names should be unique. The names are used in the stpInfos-map.
    roles = std::array<std::unique_ptr<Role>, rtt::ai::constants::MAX_ROBOT_COUNT>{
        // Roles is we play 6v6
        std::make_unique<role::Passer>("passer"),
        std::make_unique<role::PassReceiver>("receiver"),
        std::make_unique<role::Formation>("attacker_0"),
        std::make_unique<role::Formation>("attacker_1"),
        std::make_unique<role::Defender>("attacker_2"),
        std::make_unique<role::Defender>("attacker_3"),
        std::make_unique<role::Defender>("attacker_4"),
        std::make_unique<role::Defender>("attacker_5"),
        std::make_unique<role::Defender>("attacker_6"),
        std::make_unique<role::Defender>("attacker_7"),
        std::make_unique<role::Defender>("attacker_8"),

    };
}

uint8_t AttackingPass::score(const rtt::Field& field) noexcept {
    return constants::FUZZY_TRUE;
}

Dealer::FlagMap AttackingPass::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;

    flagMap.insert({"passer", {DealerFlagPriority::REQUIRED, {}, passInfo.passerId}});
    flagMap.insert({"receiver", {DealerFlagPriority::REQUIRED, {}, passInfo.receiverId}});
    flagMap.insert({"attacker_0", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"attacker_1", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"attacker_2", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"attacker_3", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"attacker_4", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"attacker_5", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"attacker_6", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"attacker_7", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"attacker_8", {DealerFlagPriority::LOW_PRIORITY, {}}});

    return flagMap;
}

void AttackingPass::calculateInfoForRoles() noexcept {
    PositionComputations::calculateInfoForAttackers(stpInfos, roles, field, world);
    PositionComputations::recalculateInfoForNonPassers(stpInfos, field, world, passInfo.receiverLocation);

    stpInfos["passer"].setPositionToShootAt(passInfo.receiverLocation);
    stpInfos["passer"].setShotPower(ShotPower::PASS);
    stpInfos["passer"].setKickOrChip(KickType::KICK);
    stpInfos["receiver"].setPositionToMoveTo(passInfo.receiverLocation);
}

bool AttackingPass::ballKicked() {
    return std::any_of(roles.begin(), roles.end(), [](const std::unique_ptr<Role>& role) {
        return role != nullptr && role->getName() == "passer" && strcmp(role->getCurrentTactic()->getName(), "Formation") == 0;
    });
}

bool AttackingPass::shouldEndPlay() noexcept {
    // If the ball is kicked, we end the play to already prepare for what happens next
    if (ballKicked()) return true;

    auto newPassInfo = stp::computations::PassComputations::calculatePass(gen::AttackingPass, world, field);
    // If the passer doesn't have the ball yet and there is a better pass available, we should stop the play
    if (stpInfos["passer"].getRobot() && !stpInfos["passer"].getRobot().value()->hasBall() &&
        newPassInfo.passScore > 1.05 * stp::PositionScoring::scorePosition(passInfo.receiverLocation, gen::AttackingPass, field, world).score)
        return true;
    // If the passer id is different, another robot can quicker get the ball, so stop
    if (newPassInfo.passerId != passInfo.passerId) {
        return true;
    }

    return false;
}

const char* AttackingPass::getName() const { return "Attacking Pass"; }
}  // namespace rtt::ai::stp::play

#include "stp/plays/referee_specific/FreeKickUsPass.h"

#include "stp/computations/PassComputations.h"
#include "stp/computations/PositionScoring.h"
#include "stp/roles/Keeper.h"
#include "stp/roles/active/FreeKickTaker.h"
#include "stp/roles/active/PassReceiver.h"
#include "stp/roles/passive/Defender.h"
#include "stp/roles/passive/Formation.h"
#include "utilities/RuntimeConfig.h"

namespace rtt::ai::stp::play {

FreeKickUsPass::FreeKickUsPass() : Play() {
    // Evaluations that have to be true in order for this play to be considered valid.
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(GlobalEvaluation::FreeKickUsGameState);

    // Evaluations that have to be true to allow the play to continue, otherwise the play will change. Plays can also end using the shouldEndPlay().
    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(GlobalEvaluation::FreeKickUsGameState);

    // Role creation, the names should be unique. The names are used in the stpInfos-map.
    roles = std::array<std::unique_ptr<Role>, rtt::ai::constants::MAX_ROBOT_COUNT>{
        // Roles is we play 6v6
        std::make_unique<role::Keeper>("keeper"),
        std::make_unique<role::FreeKickTaker>("free_kick_taker"),
        std::make_unique<role::PassReceiver>("receiver"),
        std::make_unique<role::Defender>("defender_0"),
        std::make_unique<role::Defender>("defender_1"),
        std::make_unique<role::Formation>("attacker_0"),
        // Additional roles if we play 11v11
        std::make_unique<role::Defender>("waller_0"),
        std::make_unique<role::Defender>("waller_1"),
        std::make_unique<role::Defender>("defender_2"),
        std::make_unique<role::Defender>("defender_3"),
        std::make_unique<role::Formation>("attacker_1"),
    };
}

uint8_t FreeKickUsPass::score(const rtt::Field& field) noexcept {
    passInfo = stp::computations::PassComputations::calculatePass(gen::AttackingPass, world, field);

    if (passInfo.receiverLocation == Vector2()) return 0;  // In case no pass is found

    return passInfo.passScore;
}

Dealer::FlagMap FreeKickUsPass::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {}, passInfo.keeperId}});
    flagMap.insert({"free_kick_taker", {DealerFlagPriority::REQUIRED, {}, passInfo.passerId}});
    flagMap.insert({"receiver", {DealerFlagPriority::REQUIRED, {}, passInfo.receiverId}});
    flagMap.insert({"waller_0", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"waller_1", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"defender_0", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"defender_1", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"defender_2", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"defender_3", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"attacker_0", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"attacker_1", {DealerFlagPriority::HIGH_PRIORITY, {}}});

    return flagMap;
}

void FreeKickUsPass::calculateInfoForRoles() noexcept {
    PositionComputations::calculateInfoForDefendersAndWallers(stpInfos, roles, field, world, true);
    PositionComputations::calculateInfoForAttackers(stpInfos, roles, field, world);
    PositionComputations::recalculateInfoForNonPassers(stpInfos, field, world, passInfo.receiverLocation);

    if (!ballKicked()) {
        stpInfos["receiver"].setPositionToMoveTo(passInfo.receiverLocation);
        stpInfos["free_kick_taker"].setPositionToShootAt(passInfo.receiverLocation);
        stpInfos["free_kick_taker"].setShotPower(ShotPower::PASS);
        stpInfos["free_kick_taker"].setKickOrChip(KickType::KICK);
        stpInfos["free_kick_taker"].setShootOnFirstTouch(true);
        if (RuntimeConfig::useReferee && GameStateManager::getCurrentGameState().timeLeft < 1.5) {
            stpInfos["free_kick_taker"].setPositionToShootAt(field.rightDefenseArea.rightLine().center());
            stpInfos["free_kick_taker"].setShotPower(ShotPower::MAX);
            stpInfos["free_kick_taker"].setKickOrChip(KickType::CHIP);
            stpInfos["free_kick_taker"].setShootOnFirstTouch(true);
        }
    } else {
        // Receiver goes to the receiverLocation projected on the trajectory of the ball
        auto ball = world->getWorld()->getBall()->get();
        auto ballTrajectory = LineSegment(ball->position, ball->position + ball->velocity.stretchToLength(field.playArea.width()));
        Vector2 receiverLocation = FieldComputations::projectPointToValidPositionOnLine(field, passInfo.receiverLocation, ballTrajectory.start, ballTrajectory.end);
        stpInfos["receiver"].setPositionToMoveTo(receiverLocation);

        // free_kick_taker now goes to a front grid, where the receiver is not
        if (receiverLocation.y > field.topRightGrid.getOffSetY()) {  // Receiver is going to left of the field
            stpInfos["free_kick_taker"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.bottomMidGrid, gen::SafePass, field, world));
        } else if (receiverLocation.y < field.middleMidGrid.getOffSetY()) {  // Receiver is going to right of the field
            stpInfos["free_kick_taker"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.topMidGrid, gen::SafePass, field, world));
        } else {  // Receiver is going to middle of the field- free_kick_taker will go to the closest grid on the side of the field
            auto targetGrid = stpInfos["free_kick_taker"].getRobot()->get()->getPos().y < 0 ? field.bottomMidGrid : field.topMidGrid;
            stpInfos["free_kick_taker"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, targetGrid, gen::SafePass, field, world));
        }
    }
}

bool FreeKickUsPass::ballKicked() {
    return std::any_of(roles.begin(), roles.end(), [](const std::unique_ptr<Role>& role) {
        return role != nullptr && role->getName() == "free_kick_taker" && strcmp(role->getCurrentTactic()->getName(), "Formation") == 0;
    });
}

bool FreeKickUsPass::shouldEndPlay() noexcept {
    // If the ball is kicked, we end the play to already prepare for what happens next
    if (ballKicked()) return true;

    // True if a different pass has a higher score than the current pass (by some margin)- only if the passer is not already close to the ball (since we don't want to adjust our
    // target when we're in the process of shooting
    if (stpInfos["free_kick_taker"].getRobot() && !stpInfos["free_kick_taker"].getRobot().value()->hasBall() &&
        stp::computations::PassComputations::calculatePass(gen::AttackingPass, world, field).passScore >
            1.05 * stp::PositionScoring::scorePosition(passInfo.receiverLocation, gen::AttackingPass, field, world).score &&
        (!RuntimeConfig::useReferee || GameStateManager::getCurrentGameState().timeLeft > 1.5))
        return true;

    return false;
}

const char* FreeKickUsPass::getName() const { return "Free Kick Us Pass"; }
}  // namespace rtt::ai::stp::play
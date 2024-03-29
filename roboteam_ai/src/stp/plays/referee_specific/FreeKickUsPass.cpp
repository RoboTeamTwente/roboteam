#include "stp/plays/referee_specific/FreeKickUsPass.h"

#include "stp/computations/PassComputations.h"
#include "stp/computations/PositionScoring.h"
#include "stp/roles/Keeper.h"
#include "stp/roles/active/FreeKickTaker.h"
#include "stp/roles/active/PassReceiver.h"
#include "stp/roles/passive/Defender.h"
#include "stp/roles/passive/Formation.h"

namespace rtt::ai::stp::play {

FreeKickUsPass::FreeKickUsPass() : Play() {
    // Evaluations that have to be true in order for this play to be considered valid.
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(GlobalEvaluation::FreeKickUsGameState);

    // Evaluations that have to be true to allow the play to continue, otherwise the play will change. Plays can also end using the shouldEndPlay().
    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(GlobalEvaluation::NormalOrFreeKickUsGameState);
    keepPlayEvaluation.emplace_back(GlobalEvaluation::TheyDoNotHaveBall);

    // Role creation, the names should be unique. The names are used in the stpInfos-map.
    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
        // Roles is we play 6v6
        std::make_unique<role::Keeper>("keeper"),
        std::make_unique<role::FreeKickTaker>("free_kick_taker"),
        std::make_unique<role::PassReceiver>("receiver"),
        std::make_unique<role::Defender>("defender_0"),
        std::make_unique<role::Defender>("defender_1"),
        std::make_unique<role::Formation>("attacker_0"),
        // Additional roles if we play 11v11
        std::make_unique<role::Formation>("waller_0"),
        std::make_unique<role::Formation>("waller_1"),
        std::make_unique<role::Formation>("attacker_1"),
        std::make_unique<role::Defender>("defender_2"),
        std::make_unique<role::Formation>("attacker_2"),
    };
}

uint8_t FreeKickUsPass::score(const rtt::Field& field) noexcept {
    passInfo = stp::computations::PassComputations::calculatePass(gen::AttackingPass, world, field);

    if (passInfo.passLocation == Vector2()) return 0;  // In case no pass is found

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
    flagMap.insert({"attacker_0", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"attacker_1", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"attacker_2", {DealerFlagPriority::HIGH_PRIORITY, {}}});

    return flagMap;
}

void FreeKickUsPass::calculateInfoForRoles() noexcept {
    PositionComputations::calculateInfoForDefendersAndWallers(stpInfos, roles, field, world, true);
    PositionComputations::calculateInfoForAttackers(stpInfos, roles, field, world);
    PositionComputations::recalculateInfoForNonPassers(stpInfos, field, world, passInfo.passLocation);

    if (!ballKicked()) {
        stpInfos["receiver"].setPositionToMoveTo(passInfo.passLocation);
        stpInfos["free_kick_taker"].setPositionToShootAt(passInfo.passLocation);
        stpInfos["free_kick_taker"].setShotType(ShotType::PASS);
        stpInfos["free_kick_taker"].setKickOrChip(KickOrChip::KICK);
    } else {
        // Receiver goes to the passLocation projected on the trajectory of the ball
        auto ball = world->getWorld()->getBall()->get();
        auto ballTrajectory = LineSegment(ball->position, ball->position + ball->velocity.stretchToLength(field.playArea.width()));
        Vector2 receiverLocation = FieldComputations::projectPointToValidPositionOnLine(field, passInfo.passLocation, ballTrajectory.start, ballTrajectory.end);
        stpInfos["receiver"].setPositionToMoveTo(receiverLocation);
        stpInfos["receiver"].setPidType(ball->velocity.length() > control_constants::BALL_IS_MOVING_SLOW_LIMIT ? PIDType::RECEIVE : PIDType::DEFAULT);

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
    // True if receiver has ball
    if (stpInfos["receiver"].getRobot() && stpInfos["receiver"].getRobot().value()->hasBall()) return true;

    // If the ball is moving too slow after we have kicked it, we should stop the play to get the ball
    if (ballKicked() && world->getWorld()->getBall()->get()->velocity.length() < control_constants::BALL_IS_MOVING_SLOW_LIMIT) return true;

    // True if a different pass has a higher score than the current pass (by some margin)- only if the passer is not already close to the ball (since we don't want to adjust our
    // target when we're in the process of shooting
    if (!ballKicked() && stpInfos["free_kick_taker"].getRobot() && !stpInfos["free_kick_taker"].getRobot().value()->hasBall() &&
        stp::computations::PassComputations::calculatePass(gen::AttackingPass, world, field).passScore >
            1.05 * stp::PositionScoring::scorePosition(passInfo.passLocation, gen::AttackingPass, field, world).score)
        return true;

    return false;
}

const char* FreeKickUsPass::getName() const { return "Free Kick Us Pass"; }
}  // namespace rtt::ai::stp::play
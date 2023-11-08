//
// Created by Tijmen on 22-04-22.
//

#include "stp/plays/referee_specific/FreeKickUsPass.h"

#include "stp/computations/PassComputations.h"
#include "stp/computations/PositionScoring.h"
#include "stp/roles/Keeper.h"
#include "stp/roles/active/FreeKickTaker.h"
#include "stp/roles/active/PassReceiver.h"
#include "stp/roles/passive/BallDefender.h"
#include "stp/roles/passive/Formation.h"

namespace rtt::ai::stp::play {

FreeKickUsPass::FreeKickUsPass() : Play() {
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(eval::FreeKickUsGameState);

    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(eval::NormalOrFreeKickUsGameState);
    keepPlayEvaluation.emplace_back(eval::TheyDoNotHaveBall);

    roles = std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT>{
        // Roles is we play 6v6
        std::make_unique<role::Keeper>("keeper"),
        std::make_unique<role::FreeKickTaker>("free_kick_taker"),
        std::make_unique<role::PassReceiver>("receiver"),
        std::make_unique<role::BallDefender>("defender_0"),
        std::make_unique<role::Formation>("attacker_0"),
        std::make_unique<role::BallDefender>("defender_1"),
        // Additional roles if we play 11v11
        std::make_unique<role::Formation>("waller_0"),
        std::make_unique<role::Formation>("waller_1"),
        std::make_unique<role::BallDefender>("defender_2"),
        std::make_unique<role::Formation>("waller_2"),
        std::make_unique<role::Formation>("attacker_1"),
    };
}

uint8_t FreeKickUsPass::score(const rtt::Field& field) noexcept {
    passInfo = stp::computations::PassComputations::calculatePass(gen::AttackingPass, world, field);

    if (passInfo.passLocation == Vector2()) return 0;  // In case no pass is found

    return stp::computations::PassComputations::scorePass(passInfo, world, field);
}

Dealer::FlagMap FreeKickUsPass::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {}, passInfo.keeperId}});
    flagMap.insert({"free_kick_taker", {DealerFlagPriority::REQUIRED, {}, passInfo.passerId}});
    flagMap.insert({"receiver", {DealerFlagPriority::REQUIRED, {}, passInfo.receiverId}});
    flagMap.insert({"waller_0", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"waller_1", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"waller_2", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"defender_0", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"defender_1", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"defender_2", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"attacker_0", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"attacker_1", {DealerFlagPriority::HIGH_PRIORITY, {}}});

    return flagMap;
}

void FreeKickUsPass::calculateInfoForRoles() noexcept {
    PositionComputations::calculateInfoForKeeper(stpInfos, field, world);
    PositionComputations::calculateInfoForDefendersAndWallers(stpInfos, roles, field, world);
    PositionComputations::calculateInfoForAttackers(stpInfos, roles, field, world);

    if (!ballKicked()) {
        stpInfos["receiver"].setPositionToMoveTo(passInfo.passLocation);
        stpInfos["free_kick_taker"].setPositionToShootAt(passInfo.passLocation);
        stpInfos["free_kick_taker"].setShotType(ShotType::PASS);
        stpInfos["free_kick_taker"].setKickOrChip(KickOrChip::KICK);
    } else {
        // Receiver goes to the passLocation projected on the trajectory of the ball
        auto ball = world->getWorld()->getBall()->get();
        auto ballTrajectory = LineSegment(ball->position, ball->position + ball->velocity.stretchToLength(field.playArea.width()));
        auto receiverLocation = FieldComputations::projectPointToValidPositionOnLine(field, passInfo.passLocation, ballTrajectory.start, ballTrajectory.end);
        stpInfos["receiver"].setPositionToMoveTo(receiverLocation);
        stpInfos["receiver"].setPidType(ball->velocity.length() > control_constants::BALL_IS_MOVING_SLOW_LIMIT ? PIDType::RECEIVE : PIDType::DEFAULT);

        // free_kick_taker now goes to a front grid, where the receiver is not
        if (receiverLocation.y > field.topRightGrid.getOffSetY()) {  // Receiver is going to left of the field
            stpInfos["free_kick_taker"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.bottomMidGrid, gen::BlockingPosition, field, world));
        } else if (receiverLocation.y < field.middleMidGrid.getOffSetY()) {  // Receiver is going to right of the field
            stpInfos["free_kick_taker"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.topMidGrid, gen::BlockingPosition, field, world));
        } else {  // Receiver is going to middle of the field- free_kick_taker will go to the closest grid on the side of the field
            auto targetGrid = stpInfos["free_kick_taker"].getRobot()->get()->getPos().y < 0 ? field.bottomMidGrid : field.topMidGrid;
            stpInfos["free_kick_taker"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, targetGrid, gen::BlockingPosition, field, world));
        }
    }
}

bool FreeKickUsPass::ballKicked() {
    // TODO: create better way of checking when ball has been kicked
    return std::any_of(roles.begin(), roles.end(), [](const std::unique_ptr<Role>& role) {
        return role != nullptr && role->getName() == "free_kick_taker" && strcmp(role->getCurrentTactic()->getName(), "Formation") == 0;
    });
}

bool FreeKickUsPass::shouldEndPlay() noexcept {
    // True if receiver has ball
    if (stpInfos["receiver"].getRobot() && stpInfos["receiver"].getRobot().value()->hasBall()) return true;

    // True if the free_kick_taker has shot the ball, but it is now almost stationary (pass was too soft, was reflected, etc.)
    if (ballKicked() && world->getWorld()->getBall()->get()->velocity.length() < control_constants::BALL_STILL_VEL) return true;

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
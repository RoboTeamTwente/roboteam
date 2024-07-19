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
    startPlayEvaluation.emplace_back(GlobalEvaluation::BallNotInOurDefenseAreaAndStill);
    startPlayEvaluation.emplace_back(GlobalEvaluation::WeWillHaveBall);

    // Evaluations that have to be true to allow the play to continue, otherwise the play will change. Plays can also end using the shouldEndPlay().
    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(GlobalEvaluation::NormalPlayGameState);
    keepPlayEvaluation.emplace_back(GlobalEvaluation::WeWillHaveBall);
    keepPlayEvaluation.emplace_back(GlobalEvaluation::BallNotInOurDefenseAreaAndStill);

    // Role creation, the names should be unique. The names are used in the stpInfos-map.
    roles = std::array<std::unique_ptr<Role>, rtt::ai::constants::MAX_ROBOT_COUNT>{
        // Roles is we play 6v6
        std::make_unique<role::Keeper>("keeper"),
        std::make_unique<role::Passer>("passer"),
        std::make_unique<role::PassReceiver>("receiver"),
        std::make_unique<role::Defender>("waller_0"),
        std::make_unique<role::Defender>("waller_1"),
        std::make_unique<role::Formation>("attacker_0"),
        // Additional roles if we play 11v11
        std::make_unique<role::Defender>("waller_2"),
        std::make_unique<role::Defender>("waller_3"),
        std::make_unique<role::Defender>("defender_0"),
        std::make_unique<role::Defender>("defender_1"),
        std::make_unique<role::Formation>("attacker_1"),
    };
}

uint8_t AttackingPass::score(const rtt::Field& field) noexcept {
    passInfo = stp::computations::PassComputations::calculatePass(gen::AttackingPass, world, field);

    if (passInfo.receiverLocation == Vector2()) return 0;  // In case no pass is found
    // We don't want to pass back if we are close to the goal already.
    if ((passInfo.passLocation - field.rightDefenseArea.rightLine().center()).length() < 3.5 &&
        (passInfo.receiverLocation - field.rightDefenseArea.rightLine().center()).length() > (passInfo.passLocation - field.rightDefenseArea.rightLine().center()).length())
        return 0;
    return passInfo.passScore;
}

Dealer::FlagMap AttackingPass::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {}, passInfo.keeperId}});
    flagMap.insert({"passer", {DealerFlagPriority::REQUIRED, {}, passInfo.passerId}});
    flagMap.insert({"receiver", {DealerFlagPriority::REQUIRED, {}, passInfo.receiverId}});
    for (int i = 0; i < Play::waller_count; i++) {
        if (i <= PositionComputations::amountOfWallers) {
            flagMap.insert({"waller_" + std::to_string(i), {DealerFlagPriority::HIGH_PRIORITY, {}}});
        } else
            flagMap.insert({"waller_" + std::to_string(i), {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    }
    flagMap.insert({"defender_0", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"defender_1", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"attacker_0", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"attacker_1", {DealerFlagPriority::LOW_PRIORITY, {}}});

    return flagMap;
}

void AttackingPass::calculateInfoForRoles() noexcept {
    PositionComputations::calculateInfoForDefendersAndWallers(stpInfos, roles, field, world, true);
    PositionComputations::calculateInfoForAttackers(stpInfos, roles, field, world);
    PositionComputations::recalculateInfoForNonPassers(stpInfos, field, world, passInfo.receiverLocation);

    if (!ballKicked()) {
        stpInfos["passer"].setPositionToShootAt(passInfo.receiverLocation);
        stpInfos["passer"].setShotPower(ShotPower::PASS);
        stpInfos["passer"].setKickOrChip(KickType::KICK);
        if (world->getWorld()->getBall().value() && world->getWorld()->getBall().value()->position.x > field.rightDefenseArea.leftLine().center().x) {
            stpInfos["passer"].setKickOrChip(KickType::CHIP);
            // set shot power to max
            stpInfos["passer"].setShotPower(ShotPower::MAX);
        }

        stpInfos["receiver"].setPositionToMoveTo(passInfo.receiverLocation);
    } else {
        // Receiver goes to the receiverLocation projected on the trajectory of the ball
        auto ball = world->getWorld()->getBall()->get();
        auto ballTrajectory = LineSegment(ball->position, ball->position + ball->velocity.stretchToLength(field.playArea.width()));
        Vector2 receiverLocation = FieldComputations::projectPointToValidPositionOnLine(field, passInfo.receiverLocation, ballTrajectory.start, ballTrajectory.end);
        stpInfos["receiver"].setPositionToMoveTo(receiverLocation);

        // Passer now goes to a front grid, where the receiver is not
        if (receiverLocation.y > field.topRightGrid.getOffSetY()) {  // Receiver is going to left of the field
            stpInfos["passer"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.bottomMidGrid, gen::SafePass, field, world));
        } else if (receiverLocation.y < field.middleMidGrid.getOffSetY()) {  // Receiver is going to right of the field
            stpInfos["passer"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.topMidGrid, gen::SafePass, field, world));
        } else {  // Receiver is going to middle of the field- passer will go to the closest grid on the side of the field
            auto targetGrid = stpInfos["passer"].getRobot()->get()->getPos().y < 0 ? field.bottomMidGrid : field.topMidGrid;
            stpInfos["passer"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, targetGrid, gen::SafePass, field, world));
        }
    }
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

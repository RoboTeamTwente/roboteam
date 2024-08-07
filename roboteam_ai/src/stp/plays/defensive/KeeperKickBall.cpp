#include "stp/plays/defensive/KeeperKickBall.h"

#include "stp/computations/PassComputations.h"
#include "stp/computations/PositionScoring.h"
#include "stp/roles/active/KeeperPasser.h"
#include "stp/roles/active/PassReceiver.h"
#include "stp/roles/passive/Defender.h"
#include "stp/roles/passive/Formation.h"
#include "utilities/Constants.h"

namespace rtt::ai::stp::play {

KeeperKickBall::KeeperKickBall() : Play() {
    // Evaluations that have to be true in order for this play to be considered valid.
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(GlobalEvaluation::NormalPlayGameState);
    startPlayEvaluation.emplace_back(GlobalEvaluation::BallInOurDefenseAreaAndStill);

    // Evaluations that have to be true to allow the play to continue, otherwise the play will change. Plays can also end using the shouldEndPlay().
    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(GlobalEvaluation::NormalPlayGameState);
    keepPlayEvaluation.emplace_back(GlobalEvaluation::BallInOurDefenseAreaAndStill);

    // Role creation, the names should be unique. The names are used in the stpInfos-map.
    roles = std::array<std::unique_ptr<Role>, rtt::ai::constants::MAX_ROBOT_COUNT>{
        // Roles is we play 6v6
        std::make_unique<role::KeeperPasser>("keeper"),
        std::make_unique<role::PassReceiver>("receiver"),
        std::make_unique<role::Defender>("defender_0"),
        std::make_unique<role::Defender>("defender_1"),
        std::make_unique<role::Formation>("attacker_0"),
        std::make_unique<role::Defender>("defender_2"),
        // Additional roles if we play 11v11
        std::make_unique<role::Defender>("defender_3"),
        std::make_unique<role::Formation>("attacker_1"),
        std::make_unique<role::Formation>("attacker_2"),
        std::make_unique<role::Formation>("attacker_3"),
        std::make_unique<role::Defender>("defender_4"),
    };
}

uint8_t KeeperKickBall::score(const rtt::Field& field) noexcept {
    // Calculate passInfo to be used during the play
    passInfo = stp::computations::PassComputations::calculatePass(gen::ChippingPass, world, field, true);

    // If this play is valid, the ball is in the defense area and still, and we always want to execute this play
    return constants::FUZZY_TRUE;
}

Dealer::FlagMap KeeperKickBall::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {}, passInfo.keeperId}});
    flagMap.insert({"receiver", {DealerFlagPriority::REQUIRED, {}, passInfo.receiverId}});
    flagMap.insert({"defender_0", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"defender_1", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"defender_2", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"defender_3", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"defender_4", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"attacker_0", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"attacker_1", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"attacker_2", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"attacker_3", {DealerFlagPriority::HIGH_PRIORITY, {}}});

    return flagMap;
}

void KeeperKickBall::calculateInfoForRoles() noexcept {
    PositionComputations::calculateInfoForDefendersAndWallers(stpInfos, roles, field, world, true);
    PositionComputations::calculateInfoForAttackers(stpInfos, roles, field, world);
    PositionComputations::recalculateInfoForNonPassers(stpInfos, field, world, passInfo.receiverLocation);
    stpInfos["keeper"].setShouldAvoidTheirRobots(false);
    stpInfos["keeper"].setShouldAvoidOurRobots(false);

    if (!ballKicked()) {
        stpInfos["receiver"].setPositionToMoveTo(passInfo.receiverLocation);
        stpInfos["keeper"].setPositionToShootAt(passInfo.receiverLocation);
        stpInfos["keeper"].setShotPower(ShotPower::PASS);
        stpInfos["keeper"].setKickOrChip(KickType::CHIP);
        if (stpInfos["keeper"].getRobot() && stpInfos["keeper"].getRobot()->get()->hasBall()) {
            stpInfos["keeper"].setMaxRobotVelocity(0.2);
        }
    } else {
        auto ball = world->getWorld()->getBall().value();
        // Receiver goes to the receiverLocation projected on the trajectory of the ball
        auto ballTrajectory = LineSegment(ball->position, ball->position + ball->velocity.stretchToLength(field.playArea.width()));
        Vector2 receiverLocation = FieldComputations::projectPointToValidPositionOnLine(field, passInfo.receiverLocation, ballTrajectory.start, ballTrajectory.end);
        stpInfos["receiver"].setPositionToMoveTo(receiverLocation);
    }
}

bool KeeperKickBall::ballKicked() {
    return std::any_of(roles.begin(), roles.end(), [](const std::unique_ptr<Role>& role) {
        return role != nullptr && role->getName() == "keeper" && strcmp(role->getCurrentTactic()->getName(), "Keeper Block Ball") == 0;
    });
}

bool KeeperKickBall::shouldEndPlay() noexcept {
    // If the ball is kicked, we end the play to already prepare for what happens next
    if (ballKicked()) return true;

    // If the keeper doesn't have the ball yet and there is a better pass available, we should stop the play
    if (stpInfos["keeper"].getRobot() && !stpInfos["keeper"].getRobot().value()->hasBall() &&
        stp::computations::PassComputations::calculatePass(gen::ChippingPass, world, field, true).passScore >
            1.05 * stp::PositionScoring::scorePosition(passInfo.receiverLocation, gen::ChippingPass, field, world).score)
        return true;

    return false;
}
const char* KeeperKickBall::getName() const { return "Keeper Kick Ball"; }

}  // namespace rtt::ai::stp::play

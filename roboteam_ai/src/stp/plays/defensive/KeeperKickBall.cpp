#include "stp/plays/defensive/KeeperKickBall.h"

#include "stp/computations/PassComputations.h"
#include "stp/computations/PositionScoring.h"
#include "stp/roles/active/KeeperPasser.h"
#include "stp/roles/active/PassReceiver.h"
#include "stp/roles/passive/Defender.h"
#include "stp/roles/passive/Formation.h"
#include "utilities/Constants.h"
#include "STPManager.h"
#include "rl/RLInterface.hpp"

namespace rtt::ai::stp::play {

KeeperKickBall::KeeperKickBall() : Play() {
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(GlobalEvaluation::NormalPlayGameState);
    startPlayEvaluation.emplace_back(GlobalEvaluation::BallInOurDefenseAreaAndStill);

    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(GlobalEvaluation::NormalPlayGameState);
    keepPlayEvaluation.emplace_back(GlobalEvaluation::BallInOurDefenseAreaAndStill);

    // Role creation, the names should be unique. The names are used in the stpInfos-map.
    roles = std::array<std::unique_ptr<Role>, rtt::ai::constants::MAX_ROBOT_COUNT>();

    // Create mandatory roles
    auto keeper = std::make_unique<role::KeeperPasser>("keeper");
    auto receiver = std::make_unique<role::PassReceiver>("receiver");
    
    // Move them into the array
    roles[0] = std::move(keeper);
    roles[1] = std::move(receiver);

    int currentIndex = 2;

    // Add defenders
    for (int i = 0; i < numDefenders && currentIndex < rtt::ai::constants::MAX_ROBOT_COUNT; i++) {
        auto defender = std::make_unique<role::Defender>("defender_" + std::to_string(i));
        roles[currentIndex++] = std::move(defender);
    }

    // Add attackers
    for (int i = 0; i < numAttackers && currentIndex < rtt::ai::constants::MAX_ROBOT_COUNT; i++) {
        auto attacker = std::make_unique<role::Formation>("attacker_" + std::to_string(i));
        roles[currentIndex++] = std::move(attacker);
    }
}

uint8_t KeeperKickBall::score(const rtt::Field& field) noexcept {
    passInfo = stp::computations::PassComputations::calculatePass(gen::ChippingPass, world, field, true);
    return constants::FUZZY_TRUE;
}

Dealer::FlagMap KeeperKickBall::decideRoleFlags() const noexcept {
    const_cast<KeeperKickBall*>(this)->updateRoleConfiguration();
    
    Dealer::FlagMap flagMap;

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {}, passInfo.keeperId}});
    flagMap.insert({"receiver", {DealerFlagPriority::REQUIRED, {}, passInfo.receiverId}});

    // Add defenders
    for (int i = 0; i < numDefenders; i++) {
        flagMap.insert({"defender_" + std::to_string(i), {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    }

    // Add attackers
    for (int i = 0; i < numAttackers; i++) {
        flagMap.insert({"attacker_" + std::to_string(i), {DealerFlagPriority::HIGH_PRIORITY, {}}});
    }

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

void KeeperKickBall::updateRoleConfiguration() {
    if (STPManager::isInitialized() && STPManager::getRLInterface().getIsActive()) {
        // Get suggested number of attackers from RL
        int availableSlots = rtt::ai::constants::MAX_ROBOT_COUNT - MANDATORY_ROLES;
        
        // Get and cap number of attackers
        numAttackers = STPManager::getRLInterface().getNumAttackers();
        numAttackers = std::min(numAttackers, availableSlots);
        availableSlots -= numAttackers;
        
        numDefenders = availableSlots;  // Use remaining slots for defenders
    }
    
    // Create roles array
    roles = std::array<std::unique_ptr<Role>, rtt::ai::constants::MAX_ROBOT_COUNT>();

    // Create mandatory roles first
    roles[0] = std::make_unique<role::KeeperPasser>("keeper");
    roles[1] = std::make_unique<role::PassReceiver>("receiver");

    int currentIndex = MANDATORY_ROLES;

    // Add defenders
    for (int i = 0; i < numDefenders && currentIndex < rtt::ai::constants::MAX_ROBOT_COUNT; i++) {
        roles[currentIndex++] = std::make_unique<role::Defender>("defender_" + std::to_string(i));
    }

    // Add attackers
    for (int i = 0; i < numAttackers && currentIndex < rtt::ai::constants::MAX_ROBOT_COUNT; i++) {
        roles[currentIndex++] = std::make_unique<role::Formation>("attacker_" + std::to_string(i));
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
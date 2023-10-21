//
// Created by jessevw on 17.03.20.
/// TODO-Max change to fowardPass
//

#include "stp/plays/offensive/AttackingPass.h"

#include <roboteam_utils/LineSegment.h>

#include "stp/computations/PassComputations.h"
#include "stp/computations/PositionScoring.h"
#include "stp/constants/ControlConstants.h"
#include "stp/roles/Keeper.h"
#include "stp/roles/active/PassReceiver.h"
#include "stp/roles/active/Passer.h"
#include "stp/roles/passive/BallDefender.h"
#include "stp/roles/passive/Formation.h"
#include "world/views/RobotView.hpp"
#include <roboteam_utils/Hungarian.h>

namespace rtt::ai::stp::play {
AttackingPass::AttackingPass() : Play() {
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(GlobalEvaluation::NormalPlayGameState);
    startPlayEvaluation.emplace_back(GlobalEvaluation::TheyDoNotHaveBall);
    startPlayEvaluation.emplace_back(GlobalEvaluation::BallNotInOurDefenseAreaAndStill);

    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(GlobalEvaluation::NormalPlayGameState);
    keepPlayEvaluation.emplace_back(GlobalEvaluation::TheyDoNotHaveBall);
    keepPlayEvaluation.emplace_back(GlobalEvaluation::BallNotInOurDefenseAreaAndStill);

    roles = std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT>{
        std::make_unique<role::Keeper>("keeper"),
        std::make_unique<role::Passer>("passer"),
        std::make_unique<role::PassReceiver>("receiver"),
        std::make_unique<role::BallDefender>("pass_defender_1"),
        std::make_unique<role::BallDefender>("pass_defender_2"),
        std::make_unique<role::BallDefender>("pass_defender_3"),
        std::make_unique<role::Formation>("waller_1"),
        std::make_unique<role::Formation>("waller_2"),
        std::make_unique<role::Formation>("ball_blocker"),
        std::make_unique<role::Formation>("attacker_left"),
        std::make_unique<role::Formation>("attacker_right")
    };
}

uint8_t AttackingPass::score(const rtt::Field& field) noexcept {
    passInfo = stp::computations::PassComputations::calculatePass(gen::AttackingPass, world, field);

    if (passInfo.passLocation == Vector2()) return 0;  // In case no pass is found

    return stp::computations::PassComputations::scorePass(passInfo, world, field);
}

Dealer::FlagMap AttackingPass::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag closeToOurGoalFlag(DealerFlagTitle::CLOSE_TO_OUR_GOAL, DealerFlagPriority::HIGH_PRIORITY);

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {}, passInfo.keeperId}});
    flagMap.insert({"passer", {DealerFlagPriority::REQUIRED, {}, passInfo.passerId}});
    flagMap.insert({"receiver", {DealerFlagPriority::REQUIRED, {}, passInfo.receiverId}});
    flagMap.insert({"waller_1", {DealerFlagPriority::HIGH_PRIORITY, {closeToOurGoalFlag}}});
    flagMap.insert({"waller_2", {DealerFlagPriority::HIGH_PRIORITY, {closeToOurGoalFlag}}});
    flagMap.insert({"pass_defender_1", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"pass_defender_2", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"pass_defender_3", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"ball_blocker", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"attacker_left", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"attacker_right", {DealerFlagPriority::LOW_PRIORITY, {}}});

    return flagMap;
}

void AttackingPass::calculateInfoForRoles() noexcept {
    calculateInfoForDefenders();
    calculateInfoForAttackers();
    calculateInfoForBlocker();
    calculateInfoForPassDefenders();

    /// Keeper
    stpInfos["keeper"].setPositionToMoveTo(field.leftGoalArea.rightLine().center());
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));

    /// Midfielder
    stpInfos["midfielder"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.middleMidGrid, gen::SafePosition, field, world));

    if (!ballKicked()) {
        stpInfos["receiver"].setPositionToMoveTo(passInfo.passLocation);
        stpInfos["passer"].setPositionToShootAt(passInfo.passLocation);
        stpInfos["passer"].setShotType(ShotType::PASS);
        stpInfos["passer"].setKickOrChip(KickOrChip::KICK);
    } else {
        // Receiver goes to the passLocation projected on the trajectory of the ball
        auto ball = world->getWorld()->getBall()->get();
        auto ballTrajectory = LineSegment(ball->position, ball->position + ball->velocity.stretchToLength(field.playArea.width()));
        auto receiverLocation = FieldComputations::projectPointToValidPositionOnLine(field, passInfo.passLocation, ballTrajectory.start, ballTrajectory.end);
        stpInfos["receiver"].setPositionToMoveTo(receiverLocation);
        stpInfos["receiver"].setPidType(ball->velocity.length() > control_constants::BALL_IS_MOVING_SLOW_LIMIT ? PIDType::RECEIVE : PIDType::DEFAULT);

        // Passer now goes to a front grid, where the receiver is not
        if (receiverLocation.y > field.topRightGrid.getOffSetY()) {  // Receiver is going to left of the field
            stpInfos["passer"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.bottomMidGrid, gen::BlockingPosition, field, world));
        } else if (receiverLocation.y < field.middleMidGrid.getOffSetY()) {  // Receiver is going to right of the field
            stpInfos["passer"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.topMidGrid, gen::BlockingPosition, field, world));
        } else {  // Receiver is going to middle of the field- passer will go to the closest grid on the side of the field
            auto targetGrid = stpInfos["passer"].getRobot()->get()->getPos().y < 0 ? field.bottomMidGrid : field.topMidGrid;
            stpInfos["passer"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, targetGrid, gen::BlockingPosition, field, world));
        }
    }
}

void AttackingPass::calculateInfoForDefenders() noexcept {
    constexpr auto wallerNames = std::array{"waller_1", "waller_2"};
    auto activeWallerNames = std::vector<std::string>{};
    for (auto name : wallerNames) {
        if (stpInfos[name].getRobot().has_value()) activeWallerNames.emplace_back(name);
    }

    for (int i = 0; i < activeWallerNames.size(); ++i) {
        // For each waller, stand in the right wall position and look at the ball
        auto positionToMoveTo = PositionComputations::getWallPosition(i, activeWallerNames.size(), field, world);
        auto& wallerStpInfo = stpInfos[activeWallerNames[i]];

        wallerStpInfo.setPositionToMoveTo(positionToMoveTo);
        wallerStpInfo.setAngle((world->getWorld()->getBall()->get()->position - field.leftGoalArea.rightLine().center()).angle());

        // If the waller is close to its target, ignore collisions
        constexpr double IGNORE_COLLISIONS_DISTANCE = 1.0;
        if ((wallerStpInfo.getRobot()->get()->getPos() - positionToMoveTo).length() < IGNORE_COLLISIONS_DISTANCE) {
            wallerStpInfo.setShouldAvoidOurRobots(false);
        }
    }
}

void AttackingPass::calculateInfoForBlocker() noexcept {
    stpInfos["ball_blocker"].setPositionToMoveTo(PositionComputations::getBallBlockPosition(field, world));
    if (stpInfos["ball_blocker"].getRobot())
        stpInfos["ball_blocker"].setAngle((world->getWorld()->getBall()->get()->position - stpInfos["ball_blocker"].getRobot()->get()->getPos()).toAngle());
}

void AttackingPass::calculateInfoForAttackers() noexcept {
    if (passInfo.passLocation.y > field.topRightGrid.getOffSetY()) {  // Receiver is going to left of the field
        stpInfos["attacker_left"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.middleRightGrid, gen::OffensivePosition, field, world));
        stpInfos["attacker_right"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.bottomRightGrid, gen::OffensivePosition, field, world));
    } else if (passInfo.passLocation.y < field.middleMidGrid.getOffSetY()) {  // Receiver is going to right of the field
        stpInfos["attacker_left"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.topRightGrid, gen::OffensivePosition, field, world));
        stpInfos["attacker_right"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.middleRightGrid, gen::OffensivePosition, field, world));
    } else {  // Receiver is going to middle of the field
        stpInfos["attacker_left"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.topRightGrid, gen::OffensivePosition, field, world));
        stpInfos["attacker_right"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.bottomRightGrid, gen::OffensivePosition, field, world));
    }
}

void AttackingPass::calculateInfoForPassDefenders() noexcept{
    auto enemyRobots = world->getWorld()->getThem();

    auto enemyClosestToBall = world->getWorld()->getRobotClosestToBall(world::them);

    erase_if(enemyRobots, [&](const auto enemyRobot) -> bool { return enemyClosestToBall && enemyRobot->getId() == enemyClosestToBall.value()->getId(); });

    std::map<double, Vector2> enemyMap;

    for (auto enemy : enemyRobots) {
        double score = FieldComputations::getDistanceToGoal(field, true, enemy->getPos());
        if (enemy->hasBall()) continue;
        enemyMap.insert({score, enemy->getPos()});
    }

    if (enemyMap.size() < 3) {
        stpInfos["pass_defender_1"].setPositionToDefend(
            Vector2{field.middleLeftGrid.getOffSetY() + field.middleLeftGrid.getRegionHeight() / 2, field.middleLeftGrid.getOffSetX() + field.middleLeftGrid.getRegionWidth() / 2});
        stpInfos["pass_defender_2"].setPositionToDefend(
            Vector2{field.middleMidGrid.getOffSetY() + field.middleMidGrid.getRegionHeight() / 2, field.middleMidGrid.getOffSetX() + field.middleMidGrid.getRegionWidth() / 2});
        stpInfos["pass_defender_3"].setPositionToDefend(Vector2{field.middleRightGrid.getOffSetY() + field.middleRightGrid.getRegionHeight() / 2,
                                                                field.middleRightGrid.getOffSetX() + field.middleRightGrid.getRegionWidth() / 2});
    } else {
        Vector2 enemy1 = enemyMap.begin()->second;
        stpInfos["pass_defender_1"].setPositionToDefend(enemy1);
        enemyMap.erase(enemyMap.begin());
        Vector2 enemy2 = enemyMap.begin()->second;
        stpInfos["pass_defender_2"].setPositionToDefend(enemy2);
        enemyMap.erase(enemyMap.begin());
        Vector2 enemy3 = enemyMap.begin()->second;
        stpInfos["pass_defender_3"].setPositionToDefend(enemy3);
        std::vector<Vector2> enemies = {enemy1, enemy2, enemy3};

        // Check if the pass_defenders are already assigned, if so check if the assignment is optimal
        // and if not, update the assignment. This results in the pass_defenders not switching between enemies
        // when another enemy is closer to the goal.
        if (stpInfos["pass_defender_1"].getRobot() && stpInfos["pass_defender_2"].getRobot() && stpInfos["pass_defender_3"].getRobot()) {
            std::vector<std::vector<double>> cost_matrix;
            cost_matrix.resize(3);
            for (int i = 0; i < 3; i++) {
                cost_matrix[i].resize(3);
            }

            // Calculate the distance between the pass_defenders and their enemies
            for (int i = 0; i < 3; i++) {
                cost_matrix[0][i] = (stpInfos["pass_defender_1"].getRobot()->get()->getPos() - enemies[i]).length();
                cost_matrix[1][i] = (stpInfos["pass_defender_2"].getRobot()->get()->getPos() - enemies[i]).length();
                cost_matrix[2][i] = (stpInfos["pass_defender_3"].getRobot()->get()->getPos() - enemies[i]).length();
            }

            // Calculate the optimal assignment of enemies to pass_defenders
            std::vector<int> assignments;
            rtt::Hungarian::Solve(cost_matrix, assignments);
            // Update the enemies for pass_defenders
            enemy1 = enemies[assignments[0]];
            enemy2 = enemies[assignments[1]];
            enemy3 = enemies[assignments[2]];
            stpInfos["pass_defender_1"].setPositionToDefend(enemy1);
            stpInfos["pass_defender_2"].setPositionToDefend(enemy2);
            stpInfos["pass_defender_3"].setPositionToDefend(enemy3);
        }
    }
    stpInfos["pass_defender_1"].setBlockDistance(BlockDistance::ROBOTRADIUS);
    stpInfos["pass_defender_2"].setBlockDistance(BlockDistance::ROBOTRADIUS);
    stpInfos["pass_defender_3"].setBlockDistance(BlockDistance::ROBOTRADIUS);
}

bool AttackingPass::ballKicked() {
    // TODO: create better way of checking when ball has been kicked
    return std::any_of(roles.begin(), roles.end(), [](const std::unique_ptr<Role>& role) {
        return role != nullptr && role->getName() == "passer" && strcmp(role->getCurrentTactic()->getName(), "Formation") == 0;
    });
}

bool AttackingPass::shouldEndPlay() noexcept {
    // If the receiver has the ball, the play finished successfully
    if (stpInfos["receiver"].getRobot() && stpInfos["receiver"].getRobot().value()->hasBall()) return true;

    // If the passer doesn't have the ball yet and there is a better pass available, we should stop the play
    if (!ballKicked() && stpInfos["passer"].getRobot() && !stpInfos["passer"].getRobot().value()->hasBall() &&
        stp::computations::PassComputations::calculatePass(gen::AttackingPass, world, field).passScore >
            1.05 * stp::PositionScoring::scorePosition(passInfo.passLocation, gen::AttackingPass, field, world).score)
        return true;

    return false;
}

const char* AttackingPass::getName() const { return "Attacking Pass"; }
}  // namespace rtt::ai::stp::play

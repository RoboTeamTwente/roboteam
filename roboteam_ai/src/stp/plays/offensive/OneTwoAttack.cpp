//
// Created by jibbe on 5-6-23.
//
#include "stp/plays/offensive/OneTwoAttack.hpp"

#include "stp/roles/Keeper.h"
#include "stp/roles/active/Assistant.hpp"
#include "stp/roles/active/Passer.h"
#include "stp/roles/passive/BallDefender.h"
#include "stp/roles/passive/Formation.h"
#include "stp/roles/passive/RobotDefender.h"

namespace rtt::ai::stp::play {
    OneTwoAttack::OneTwoAttack() : Play() {
        //start play evaluations
        startPlayEvaluation.clear();
        startPlayEvaluation.emplace_back(GlobalEvaluation::NormalPlayGameState);
        startPlayEvaluation.emplace_back(GlobalEvaluation::TheyDoNotHaveBall);
        startPlayEvaluation.emplace_back(GlobalEvaluation::BallNotInOurDefenseAreaAndStill);
        startPlayEvaluation.emplace_back(GlobalEvaluation::NoGoalVisionFromBall);

        //keep play evaluations
        keepPlayEvaluation.clear();
        keepPlayEvaluation.emplace_back(GlobalEvaluation::NormalPlayGameState);
        keepPlayEvaluation.emplace_back(GlobalEvaluation::TheyDoNotHaveBall);
        keepPlayEvaluation.emplace_back(GlobalEvaluation::BallNotInOurDefenseAreaAndStill);

        //roles
        roles = std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT>{
            std::make_unique<role::Passer>("striker"),
            std::make_unique<role::Assistant>("assistant"),
            std::make_unique<role::Keeper>("keeper"),
            std::make_unique<role::RobotDefender>("robot_defender"),
            std::make_unique<role::BallDefender>("ball_defender_1"),
            std::make_unique<role::BallDefender>("ball_defender_2"),
            std::make_unique<role::BallDefender>("ball_defender_3"),
            std::make_unique<role::Formation>("waller_1"),
            std::make_unique<role::Formation>("waller_2"),
            std::make_unique<role::Formation>("waller_3"),
            std::make_unique<role::Formation>("waller_4")
        };
    }

    uint8_t OneTwoAttack::score(const Field& field) noexcept {
        firstPassInfo = computations::PassComputations::calculatePass(gen::SafePass, world, field);
        secondPassInfo = computations::PassComputations::calculatePass(gen::OffensivePosition, world, field);
        return 0;
    }

    Dealer::FlagMap OneTwoAttack::decideRoleFlags() const noexcept {
        Dealer::FlagMap flagMap;
        Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER, DealerFlagPriority::KEEPER);
        Dealer::DealerFlag kickerFlag(DealerFlagTitle::CAN_KICK_BALL, DealerFlagPriority::REQUIRED);
        Dealer::DealerFlag detectBallFlag(DealerFlagTitle::CAN_DETECT_BALL, DealerFlagPriority::HIGH_PRIORITY);
        Dealer::DealerFlag closeToBallFlag(DealerFlagTitle::CLOSEST_TO_BALL, DealerFlagPriority::MEDIUM_PRIORITY);
        Dealer::DealerFlag closeToTheirGoalFlag(DealerFlagTitle::CLOSE_TO_THEIR_GOAL, DealerFlagPriority::HIGH_PRIORITY);
        Dealer::DealerFlag closeToOurGoalFlag(DealerFlagTitle::CLOSE_TO_OUR_GOAL, DealerFlagPriority::HIGH_PRIORITY);

        flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {keeperFlag}}});
        flagMap.insert({"striker", {DealerFlagPriority::REQUIRED, {kickerFlag, detectBallFlag, closeToBallFlag}}});
        flagMap.insert({"assistant", {DealerFlagPriority::REQUIRED, {kickerFlag, detectBallFlag, closeToTheirGoalFlag}}});
        flagMap.insert({"waller_1", {DealerFlagPriority::HIGH_PRIORITY, {closeToOurGoalFlag}}});
        flagMap.insert({"waller_2", {DealerFlagPriority::HIGH_PRIORITY, {closeToOurGoalFlag}}});
        flagMap.insert({"waller_3", {DealerFlagPriority::HIGH_PRIORITY, {closeToOurGoalFlag}}});
        flagMap.insert({"waller_4", {DealerFlagPriority::HIGH_PRIORITY, {closeToOurGoalFlag}}});
        flagMap.insert({"robot_defender", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
        flagMap.insert({"ball_defender_1", {DealerFlagPriority::LOW_PRIORITY, {}}});
        flagMap.insert({"ball_defender_2", {DealerFlagPriority::LOW_PRIORITY, {}}});
        flagMap.insert({"ball_defender_3", {DealerFlagPriority::LOW_PRIORITY, {}}});

        return flagMap;
    }
    void OneTwoAttack::calculateInfoForRoles() noexcept {
        calculateInfoForKeeper();
        calculateInfoForStriker();
        calculateInfoForAssistant();
        calculateInfoForWallers();
        calculateInfoForDefenders();
    }

    const char* OneTwoAttack::getName() { return "OneTwoAttack"; }

    bool OneTwoAttack::shouldEndPlay() noexcept {
        return std::any_of(roles.begin(), roles.end(), [](const std::unique_ptr<Role>& role) { return role != nullptr && role->getName() == "striker" && role->finished(); });
    }

    void OneTwoAttack::calculateInfoForKeeper() noexcept {
        stpInfos["keeper"].setPositionToMoveTo(field.leftGoalArea.rightLine().center());
        if(!world->getWorld()->getThem().empty()){
            stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));
        }
        else{
            stpInfos["keeper"].setPositionToDefend(Vector2(0, 0));
        }
    }

    void OneTwoAttack::calculateInfoForStriker() noexcept {
        stpInfos["striker"].setPositionToMoveTo(world->getWorld()->getBall()->get()->position);
        stpInfos["striker"].setPositionToShootAt(firstPassInfo.passLocation);
        stpInfos["striker"].setKickOrChip(KickOrChip::KICK);
        stpInfos["striker"].setShotType(ShotType::PASS);
    }

    void OneTwoAttack::calculateInfoForAssistant() noexcept {
        stpInfos["assistant"].setPositionToMoveTo(firstPassInfo.passLocation);
        stpInfos["assistant"].setPositionToShootAt(secondPassInfo.passLocation);
    }

    void OneTwoAttack::calculateInfoForWallers() noexcept {
        constexpr auto wallerNames = std::array{"waller_1", "waller_2", "waller_3", "waller_4"};
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

    void OneTwoAttack::calculateInfoForDefenders() noexcept {
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
            stpInfos["ball_defender_1"].setPositionToDefend(Vector2{field.middleLeftGrid.getOffSetY() + field.middleLeftGrid.getRegionHeight() / 2, field.middleLeftGrid.getOffSetX() + field.middleLeftGrid.getRegionWidth() / 2});
            stpInfos["ball_defender_2"].setPositionToDefend(Vector2{field.middleMidGrid.getOffSetY() + field.middleMidGrid.getRegionHeight() / 2, field.middleMidGrid.getOffSetX() + field.middleMidGrid.getRegionWidth() / 2});
            stpInfos["ball_defender_3"].setPositionToDefend(Vector2{field.middleRightGrid.getOffSetY() + field.middleRightGrid.getRegionHeight() / 2, field.middleRightGrid.getOffSetX() + field.middleRightGrid.getRegionWidth() / 2});
        }
        else {
            stpInfos["ball_defender_1"].setPositionToDefend(enemyMap.begin()->second);
            enemyMap.erase(enemyMap.begin());
            stpInfos["ball_defender_2"].setPositionToDefend(enemyMap.begin()->second);
            enemyMap.erase(enemyMap.begin());
            stpInfos["ball_defender_3"].setPositionToDefend(enemyMap.begin()->second);
        }
        stpInfos["ball_defender_1"].setBlockDistance(BlockDistance::ROBOTRADIUS);
        stpInfos["ball_defender_2"].setBlockDistance(BlockDistance::ROBOTRADIUS);
        stpInfos["ball_defender_3"].setBlockDistance(BlockDistance::ROBOTRADIUS);

        if (!enemyClosestToBall.has_value()) {
            for (auto &role : roles){
                if (role->getName() == "robot_defender") {
                    role = std::make_unique<role::BallDefender>(role::BallDefender("robot_defender"));
                    break;
                }
            }
        } else {
            stpInfos["robot_defender"].setEnemyRobot(enemyClosestToBall);
        }

        stpInfos["robot_defender"].setPositionToDefend(field.leftGoalArea.rightLine().center());
        stpInfos["robot_defender"].setBlockDistance(BlockDistance::ROBOTRADIUS);
    }
    }

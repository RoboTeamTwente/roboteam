//
// Created by floris on 15-11-20.
//
#include "control/positionControl/BBTrajectories/WorldObjects.h"

#include <algorithm>

#include "stp/constants/ControlConstants.h"
#include "world/World.hpp"
#include "world/WorldData.hpp"

namespace rtt::BB {

WorldObjects::WorldObjects() = default;

std::optional<CollisionData> WorldObjects::getFirstCollision(const rtt::world::World *world, const rtt::Field &field, const Trajectory2D &Trajectory,
                                                             const std::unordered_map<int, std::vector<Vector2>> &computedPaths, int robotId, ai::stp::AvoidObjects avoidObjects,
                                                             std::unordered_map<int, int> completedTimeSteps) {
    double timeStep = 0.1;
    auto pathPoints = Trajectory.getPathApproach(timeStep);

    std::vector<CollisionData> collisionDatas;

    if (avoidObjects.shouldAvoidOutOfField) {
        calculateFieldCollisions(field, collisionDatas, pathPoints, timeStep, completedTimeSteps[robotId]);
    }
    if (avoidObjects.shouldAvoidDefenseArea) {
        calculateDefenseAreaCollisions(field, collisionDatas, pathPoints, robotId, timeStep, completedTimeSteps[robotId]);
    }
    if (avoidObjects.shouldAvoidBall) {
        calculateBallCollisions(world, collisionDatas, pathPoints, timeStep, avoidObjects.avoidBallDist, completedTimeSteps[robotId]);
    }
    if (avoidObjects.shouldAvoidTheirRobots || avoidObjects.notAvoidTheirRobotId != -1) {
        calculateEnemyRobotCollisions(world, collisionDatas, pathPoints, timeStep, completedTimeSteps[robotId], avoidObjects.notAvoidTheirRobotId);
    }
    if (avoidObjects.shouldAvoidOurRobots) {
        calculateOurRobotCollisions(world, collisionDatas, pathPoints, computedPaths, robotId, timeStep, completedTimeSteps);
    }
    if (rtt::ai::GameStateManager::getCurrentGameState().getStrategyName() == "ball_placement_them") {
        calculateBallPlacementCollision(world, collisionDatas, pathPoints, timeStep, completedTimeSteps[robotId]);
    }
    if (!collisionDatas.empty()) {
        return collisionDatas[0];
    } else {
        return std::nullopt;
    }
}

std::optional<CollisionData> WorldObjects::getFirstDefenseAreaCollision(const rtt::Field &field, const Trajectory2D &Trajectory,
                                                                        const std::unordered_map<int, std::vector<Vector2>> &computedPaths, int robotId) {
    double timeStep = 0.1;
    auto pathPoints = Trajectory.getPathApproach(timeStep);

    std::vector<CollisionData> collisionDatas;
    calculateDefenseAreaCollisions(field, collisionDatas, pathPoints, robotId, timeStep, 0);

    if (!collisionDatas.empty()) {
        return collisionDatas[0];
    } else {
        return std::nullopt;
    }
}

void WorldObjects::calculateFieldCollisions(const rtt::Field &field, std::vector<CollisionData> &collisionDatas, const std::vector<Vector2> &pathPoints, double timeStep,
                                            size_t completedTimeSteps) {
    double robotRadius = rtt::ai::Constants::ROBOT_RADIUS();
    for (size_t i = completedTimeSteps; i < pathPoints.size(); i++) {
        if (!field.playArea.contains(pathPoints[i], robotRadius)) {
            // Don't care about the field if the robot is already outside the field (i == 0 is the first point of the robot's path, so almost the currentPosition).
            if (i == 0) return;

            insertCollisionData(collisionDatas, CollisionData{pathPoints[i], timeStep * (i - completedTimeSteps), BB::CollisionType::FIELD});
            return;
        }
    }
}

void WorldObjects::calculateDefenseAreaCollisions(const rtt::Field &field, std::vector<CollisionData> &collisionDatas, const std::vector<Vector2> &pathPoints, int robotId,
                                                  double timeStep, size_t completedTimeSteps) {
    auto [theirDefenseAreaMargin, ourDefenseAreaMargin] = rtt::ai::FieldComputations::getDefenseAreaMargin();
    const auto &ourDefenseArea = rtt::ai::FieldComputations::getDefenseArea(field, true, ourDefenseAreaMargin, 1);
    const auto &theirDefenseArea = rtt::ai::FieldComputations::getDefenseArea(field, false, theirDefenseAreaMargin, 1);

    for (size_t i = completedTimeSteps; i < pathPoints.size(); i++) {
        if (ourDefenseArea.contains(pathPoints[i]) || theirDefenseArea.contains(pathPoints[i])) {
            // Don't care about the defense area if the robot is already in the defense area. It should just get out as fast as possible :)
            // if (i == 0) return;

            insertCollisionData(collisionDatas, CollisionData{pathPoints[i], timeStep * (i - completedTimeSteps), BB::CollisionType::DEFENSEAREA});
            return;
        }
    }
}

void WorldObjects::calculateBallCollisions(const rtt::world::World *world, std::vector<CollisionData> &collisionDatas, const std::vector<Vector2> &pathPoints, double timeStep,
                                           double dist, size_t completedTimeSteps) {
    double ballAvoidanceTime = 0.7;

    const Vector2 &startPositionBall = world->getWorld()->getBall()->get()->position;
    const Vector2 &VelocityBall = world->getWorld()->getBall()->get()->velocity;

    for (double loopTime = 0; loopTime < ballAvoidanceTime; loopTime += timeStep, completedTimeSteps++) {
        if (completedTimeSteps + 1 >= pathPoints.size()) {
            return;
        }
        if ((pathPoints[completedTimeSteps] - (startPositionBall + VelocityBall * loopTime)).length() > 1) {
            continue;
        }
        const Vector2 &currentPoint = pathPoints[completedTimeSteps];
        const Vector2 &nextPoint = pathPoints[completedTimeSteps + 1];
        const double startPointX_R1 = currentPoint.x;
        const double startPointY_R1 = currentPoint.y;
        const double startPointX_ball = startPositionBall.x;
        const double startPointY_ball = startPositionBall.y;

        const double velocityX_R1 = (nextPoint.x - startPointX_R1) / timeStep;
        const double velocityY_R1 = (nextPoint.y - startPointY_R1) / timeStep;
        const double velocityX_ball = VelocityBall.x;
        const double velocityY_ball = VelocityBall.y;

        const double velocityX_diff = velocityX_R1 - velocityX_ball;
        const double velocityY_diff = velocityY_R1 - velocityY_ball;

        double minTime = (startPointX_R1 * velocityX_diff + velocityX_R1 * startPointX_ball - startPointX_ball * velocityX_ball - startPointY_R1 * velocityY_R1 +
                          startPointY_R1 * velocityY_ball + velocityY_R1 * startPointY_ball - startPointY_ball * velocityY_ball) /
                         (velocityX_R1 * velocityX_R1 - 2 * velocityX_R1 * velocityX_ball + velocityX_ball * velocityX_ball + velocityY_diff * velocityY_diff);
        minTime = std::clamp(minTime, 0.0, timeStep);

        Vector2 ourLocationAtMinTime = Vector2(startPointX_R1, startPointY_R1) + Vector2(velocityX_R1, velocityY_R1) * minTime;
        Vector2 theirLocationAtMinTime = Vector2(startPointX_ball, startPointY_ball) + Vector2(velocityX_ball, velocityY_ball) * minTime;
        double distance = (ourLocationAtMinTime - theirLocationAtMinTime).length();

        if (distance < dist) {
            insertCollisionData(collisionDatas, CollisionData{pathPoints[completedTimeSteps], loopTime, BB::CollisionType::BALL});
            return;
        }
    }
}

void WorldObjects::calculateEnemyRobotCollisions(const rtt::world::World *world, std::vector<CollisionData> &collisionDatas, const std::vector<Vector2> &pathPoints,
                                                 double timeStep, size_t completedTimeSteps, int avoidId) {
    const std::vector<world::view::RobotView> &theirRobots = world->getWorld()->getThem();
    const double maxCollisionCheckTime = 0.7;
    const double avoidanceDistance = 4 * ai::Constants::ROBOT_RADIUS_MAX();

    for (double loopTime = 0; loopTime < maxCollisionCheckTime; loopTime += timeStep, completedTimeSteps++) {
        if (completedTimeSteps + 1 >= pathPoints.size()) {
            return;
        }

        const Vector2 &currentPoint = pathPoints[completedTimeSteps];
        const Vector2 &nextPoint = pathPoints[completedTimeSteps + 1];
        const double startPointX_OurRobot = currentPoint.x;
        const double startPointY_OurRobot = currentPoint.y;
        const double velocityX_OurRobot = (nextPoint.x - startPointX_OurRobot) / timeStep;
        const double velocityY_OurRobot = (nextPoint.y - startPointY_OurRobot) / timeStep;

        for (const auto &opponentRobot : theirRobots) {
            const double startPointX_OpponentRobot = opponentRobot->getPos().x + opponentRobot->getVel().x * loopTime;
            const double startPointY_OpponentRobot = opponentRobot->getPos().y + opponentRobot->getVel().y * loopTime;
            const double velocityX_OpponentRobot = opponentRobot->getVel().x;
            const double velocityY_OpponentRobot = opponentRobot->getVel().y;

            const double velocityX_diff = velocityX_OurRobot - velocityX_OpponentRobot;
            const double velocityY_diff = velocityY_OurRobot - velocityY_OpponentRobot;
            if (velocityX_diff * velocityX_diff + velocityY_diff * velocityY_diff < 1) {
                continue;
            }

            double minTime = (startPointX_OurRobot * velocityX_diff + velocityX_OurRobot * startPointX_OpponentRobot - startPointX_OpponentRobot * velocityX_OpponentRobot -
                              startPointY_OurRobot * velocityY_OurRobot + startPointY_OurRobot * velocityY_OpponentRobot + velocityY_OurRobot * startPointY_OpponentRobot -
                              startPointY_OpponentRobot * velocityY_OpponentRobot) /
                             (velocityX_OurRobot * velocityX_OurRobot - 2 * velocityX_OurRobot * velocityX_OpponentRobot + velocityX_OpponentRobot * velocityX_OpponentRobot +
                              velocityY_diff * velocityY_diff);
            minTime = std::clamp(minTime, 0.0, timeStep);

            const Vector2 ourLocationAtMinTime = Vector2(startPointX_OurRobot, startPointY_OurRobot) + Vector2(velocityX_OurRobot, velocityY_OurRobot) * minTime;
            const Vector2 theirLocationAtMinTime =
                Vector2(startPointX_OpponentRobot, startPointY_OpponentRobot) + Vector2(velocityX_OpponentRobot, velocityY_OpponentRobot) * minTime;
            const double distance = (ourLocationAtMinTime - theirLocationAtMinTime).length();
            if (distance < avoidanceDistance) {
                insertCollisionData(collisionDatas, CollisionData{ourLocationAtMinTime, loopTime, BB::CollisionType::ENEMYROBOT});
                return;
            }
        }
    }
}

void WorldObjects::calculateOurRobotCollisions(const rtt::world::World *world, std::vector<CollisionData> &collisionDatas, const std::vector<Vector2> &pathPoints,
                                               const std::unordered_map<int, std::vector<Vector2>> &computedPaths, int robotId, double timeStep,
                                               std::unordered_map<int, int> completedTimeSteps) {
    const auto ourRobots = world->getWorld()->getUs();
    const double maxCollisionCheckTime = 0.7;
    const double avoidanceDistance = 2 * ai::Constants::ROBOT_RADIUS() + 2 * ai::stp::control_constants::GO_TO_POS_ERROR_MARGIN;

    for (double loopTime = 0; loopTime < maxCollisionCheckTime; loopTime += timeStep, completedTimeSteps[robotId]++) {
        if (completedTimeSteps[robotId] + 1 >= pathPoints.size()) {
            return;
        }
        const double startPointX_OurRobot = pathPoints[completedTimeSteps[robotId]].x;
        const double startPointY_OurRobot = pathPoints[completedTimeSteps[robotId]].y;
        const double velocityX_OurRobot = (pathPoints[completedTimeSteps[robotId] + 1].x - startPointX_OurRobot) / timeStep;
        const double velocityY_OurRobot = (pathPoints[completedTimeSteps[robotId] + 1].y - startPointY_OurRobot) / timeStep;

        for (const auto &ourRobot : ourRobots) {
            const int otherRobotId = ourRobot->getId();

            if (otherRobotId == robotId) continue;
            if ((pathPoints[completedTimeSteps[robotId]] - (ourRobot->getPos() + ourRobot->getVel() * loopTime)).length() > 1) {
                continue;
            }

            const auto completedTimeStepsIt = completedTimeSteps.find(otherRobotId);
            const auto computedPathsIt = computedPaths.find(otherRobotId);

            double startPointX_OtherRobot, startPointY_OtherRobot, velocityX_OtherRobot, velocityY_OtherRobot;

            if (completedTimeStepsIt != completedTimeSteps.end() && computedPathsIt != computedPaths.end() && completedTimeStepsIt->second + 1 < computedPathsIt->second.size()) {
                startPointX_OtherRobot = computedPathsIt->second[completedTimeStepsIt->second].x;
                startPointY_OtherRobot = computedPathsIt->second[completedTimeStepsIt->second].y;
                velocityX_OtherRobot = (computedPathsIt->second[completedTimeStepsIt->second + 1].x - startPointX_OtherRobot) / timeStep;
                velocityY_OtherRobot = (computedPathsIt->second[completedTimeStepsIt->second + 1].y - startPointY_OtherRobot) / timeStep;
            } else {
                if (computedPathsIt == computedPaths.end() || computedPathsIt->second.empty()) {
                    startPointX_OtherRobot = ourRobot->getPos().x;
                    startPointY_OtherRobot = ourRobot->getPos().y;
                } else {
                    startPointX_OtherRobot = computedPathsIt->second.back().x;
                    startPointY_OtherRobot = computedPathsIt->second.back().y;
                }
                velocityX_OtherRobot = 0;
                velocityY_OtherRobot = 0;
            }
            const double velocityX_diff = velocityX_OurRobot - velocityX_OtherRobot;
            const double velocityY_diff = velocityY_OurRobot - velocityY_OtherRobot;
            if (velocityX_diff * velocityX_diff + velocityY_diff * velocityY_diff < 1) {
                continue;
            }

            double minTime = (startPointX_OurRobot * velocityX_diff + velocityX_OurRobot * startPointX_OtherRobot - startPointX_OtherRobot * velocityX_OtherRobot -
                              startPointY_OurRobot * velocityY_OurRobot + startPointY_OurRobot * velocityY_OtherRobot + velocityY_OurRobot * startPointY_OtherRobot -
                              startPointY_OtherRobot * velocityY_OtherRobot) /
                             (velocityX_OurRobot * velocityX_OurRobot - 2 * velocityX_OurRobot * velocityX_OtherRobot + velocityX_OtherRobot * velocityX_OtherRobot +
                              velocityY_diff * velocityY_diff);
            minTime = std::clamp(minTime, 0.0, timeStep);

            const Vector2 ourLocationAtMinTime = Vector2(startPointX_OurRobot, startPointY_OurRobot) + Vector2(velocityX_OurRobot, velocityY_OurRobot) * minTime;
            const Vector2 otherRobotLocationAtMinTime = Vector2(startPointX_OtherRobot, startPointY_OtherRobot) + Vector2(velocityX_OtherRobot, velocityY_OtherRobot) * minTime;
            const double distanceSquared = (ourLocationAtMinTime - otherRobotLocationAtMinTime).length();
            if (distanceSquared < avoidanceDistance) {
                insertCollisionData(collisionDatas, CollisionData{ourLocationAtMinTime, loopTime, BB::CollisionType::OWNROBOT});
                return;
            }
        }
    }
}

void WorldObjects::calculateBallPlacementCollision(const rtt::world::World *world, std::vector<CollisionData> &collisionDatas, const std::vector<Vector2> &pathPoints,
                                                   double timeStep, size_t completedTimeSteps) {
    auto ballPlacementPos = rtt::ai::GameStateManager::getRefereeDesignatedPosition();
    auto startPositionBall = world->getWorld()->getBall()->get()->position;
    LineSegment ballPlacementLine(startPositionBall, ballPlacementPos);
    double time = completedTimeSteps * timeStep;

    for (size_t i = completedTimeSteps; i < pathPoints.size(); i++) {
        if (ballPlacementLine.distanceToLine(pathPoints[i]) < ai::stp::control_constants::AVOID_BALL_DISTANCE) {
            insertCollisionData(collisionDatas, CollisionData{pathPoints[i], timeStep * (i - completedTimeSteps), BB::CollisionType::BALLPLACEMENT});
            return;
        }
    }
}

// Insert a collision data object into a vector of collision data objects, sorted by collision time
void WorldObjects::insertCollisionData(std::vector<CollisionData> &collisionDatas, const CollisionData &collisionData) {
    collisionDatas.insert(std::upper_bound(collisionDatas.begin(), collisionDatas.end(), collisionData,
                                           [](CollisionData const &data, CollisionData const &compare) { return data.collisionTime < compare.collisionTime; }),
                          collisionData);
}
}  // namespace rtt::BB
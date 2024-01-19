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
                                                             const std::unordered_map<int, std::vector<Vector2>> &computedPaths, int robotId, ai::stp::AvoidObjects avoidObjects) {
    // Set the time step for the trajectory calculation
    double timeStep = 0.1;

    // Calculate the points on the trajectory at intervals of `timeStep`
    auto pathPoints = Trajectory.getPathApproach(timeStep);

    // If the computed path for the robot exists and has more than 10 points,
    // and `pathPoints` also has more than 10 points, remove the last 3 points from `pathPoints`.
    // This avoids checking for collisions at the endpoint of the trajectory.
    if (computedPaths.contains(robotId) && computedPaths.at(robotId).size() > 10 && pathPoints.size() > 10) {
        pathPoints.erase(pathPoints.end() - 3, pathPoints.end());
    }

    // Calculate the number of time steps that have been completed
    size_t completedTimeSteps = computedPaths.contains(robotId) && pathPoints.size() > computedPaths.at(robotId).size() ? pathPoints.size() - computedPaths.at(robotId).size() : 0;

    std::vector<CollisionData> collisionDatas;

    if (avoidObjects.shouldAvoidOutOfField) {
        calculateFieldCollisions(field, collisionDatas, pathPoints, timeStep, completedTimeSteps);
    }
    if (avoidObjects.shouldAvoidDefenseArea) {
        calculateDefenseAreaCollisions(field, collisionDatas, pathPoints, robotId, timeStep, completedTimeSteps);
    }
    if (avoidObjects.shouldAvoidBall) {
        calculateBallCollisions(world, collisionDatas, pathPoints, timeStep, avoidObjects.avoidBallDist, completedTimeSteps);
    }
    if (avoidObjects.shouldAvoidTheirRobots || avoidObjects.notAvoidTheirRobotId != -1) {
        calculateEnemyRobotCollisions(world, collisionDatas, pathPoints, timeStep, completedTimeSteps, avoidObjects.notAvoidTheirRobotId);
    }
    if (avoidObjects.shouldAvoidOurRobots) {
        calculateOurRobotCollisions(world, collisionDatas, pathPoints, computedPaths, robotId, timeStep, completedTimeSteps);
    }
    if (rtt::ai::GameStateManager::getCurrentGameState().getStrategyName() == "ball_placement_them") {
        calculateBallPlacementCollision(world, collisionDatas, pathPoints, timeStep, completedTimeSteps);
    }
    if (!collisionDatas.empty()) {
        return collisionDatas[0];
    } else {
        return std::nullopt;
    }
}

std::optional<CollisionData> WorldObjects::getFirstDefenseAreaCollision(const rtt::Field &field, const Trajectory2D &Trajectory,
                                                                        const std::unordered_map<int, std::vector<Vector2>> &computedPaths, int robotId) {
    // Set the time step for the trajectory calculation
    double timeStep = 0.1;

    // Calculate the points on the trajectory at intervals of `timeStep`
    auto pathPoints = Trajectory.getPathApproach(timeStep);

    // If the computed path for the robot exists and has more than 10 points,
    // and `pathPoints` also has more than 10 points, remove the last 3 points from `pathPoints`.
    // This avoids checking for collisions at the endpoint of the trajectory.
    if (computedPaths.contains(robotId) && computedPaths.at(robotId).size() > 10 && pathPoints.size() > 10) {
        pathPoints.erase(pathPoints.end() - 3, pathPoints.end());
    }

    // Calculate the number of time steps that have been completed
    size_t completedTimeSteps = computedPaths.contains(robotId) && pathPoints.size() > computedPaths.at(robotId).size() ? pathPoints.size() - computedPaths.at(robotId).size() : 0;

    std::vector<CollisionData> collisionDatas;

    calculateDefenseAreaCollisions(field, collisionDatas, pathPoints, robotId, timeStep, completedTimeSteps);

    if (!collisionDatas.empty()) {
        return collisionDatas[0];
    } else {
        return std::nullopt;
    }
}

void WorldObjects::calculateFieldCollisions(const rtt::Field &field, std::vector<CollisionData> &collisionDatas, const std::vector<Vector2> &pathPoints, double timeStep,
                                            size_t completedTimeSteps) {
    for (size_t i = completedTimeSteps; i < pathPoints.size(); i++) {
        if (!field.playArea.contains(pathPoints[i], rtt::ai::Constants::ROBOT_RADIUS())) {
            // Don't care about the field if the robot is already outside the field (i == 0 is the first point of the robot's path, so almost the currentPosition).
            if (i == 0) return;

            insertCollisionData(collisionDatas, CollisionData{pathPoints[i], timeStep * (i - completedTimeSteps), "FieldCollision"});
            return;
        }
    }
}

void WorldObjects::calculateDefenseAreaCollisions(const rtt::Field &field, std::vector<CollisionData> &collisionDatas, const std::vector<Vector2> &pathPoints, int robotId,
                                                  double timeStep, size_t completedTimeSteps) {
    auto [theirDefenseAreaMargin, ourDefenseAreaMargin] = rtt::ai::FieldComputations::getDefenseAreaMargin();
    auto ourDefenseArea = rtt::ai::FieldComputations::getDefenseArea(field, true, ourDefenseAreaMargin, 1);
    auto theirDefenseArea = rtt::ai::FieldComputations::getDefenseArea(field, false, theirDefenseAreaMargin, 1);

    for (size_t i = completedTimeSteps; i < pathPoints.size(); i++) {
        if (ourDefenseArea.contains(pathPoints[i]) || theirDefenseArea.contains(pathPoints[i])) {
            // Don't care about the defense area if the robot is already in the defense area. It should just get out as fast as possible :)
            // if (i == 0) return;

            insertCollisionData(collisionDatas, CollisionData{pathPoints[i], timeStep * (i - completedTimeSteps), "DefenseAreaCollision"});
            return;
        }
    }
}

void WorldObjects::calculateBallCollisions(const rtt::world::World *world, std::vector<CollisionData> &collisionDatas, std::vector<Vector2> pathPoints, double timeStep,
                                           double dist, size_t completedTimeSteps) {
    double ballAvoidanceTime = 0.5;

    auto startPositionBall = world->getWorld()->getBall()->get()->position;
    auto VelocityBall = world->getWorld()->getBall()->get()->velocity;

    for (double loopTime = 0; loopTime < ballAvoidanceTime; loopTime += timeStep, completedTimeSteps++) {
        if (completedTimeSteps + 1 >= pathPoints.size()) {
            return;
        }
        if ((pathPoints[completedTimeSteps] - (startPositionBall + VelocityBall * loopTime)).length() > 1) {
            continue;
        }
        double startPointX_R1 = pathPoints[completedTimeSteps].x;
        double startPointY_R1 = pathPoints[completedTimeSteps].y;
        double startPointX_ball = startPositionBall.x;
        double startPointY_ball = startPositionBall.y;

        double velocityX_R1 = (pathPoints[completedTimeSteps + 1].x - startPointX_R1) / timeStep;
        double velocityY_R1 = (pathPoints[completedTimeSteps + 1].y - startPointY_R1) / timeStep;
        double velocityX_ball = VelocityBall.x;
        double velocityY_ball = VelocityBall.y;

        double velocityX_diff = velocityX_R1 - velocityX_ball;
        double velocityY_diff = velocityY_R1 - velocityY_ball;

        double minTime = (startPointX_R1 * velocityX_diff + velocityX_R1 * startPointX_ball - startPointX_ball * velocityX_ball - startPointY_R1 * velocityY_R1 +
                          startPointY_R1 * velocityY_ball + velocityY_R1 * startPointY_ball - startPointY_ball * velocityY_ball) /
                         (velocityX_R1 * velocityX_R1 - 2 * velocityX_R1 * velocityX_ball + velocityX_ball * velocityX_ball + velocityY_diff * velocityY_diff);
        if (minTime < 0) {
            minTime = 0;
        }
        if (minTime > timeStep) {
            minTime = timeStep;
        }

        // check distance at minTime
        Vector2 ourLocationAtMinTime = Vector2(startPointX_R1, startPointY_R1) + Vector2(velocityX_R1, velocityY_R1) * minTime;
        Vector2 theirLocationAtMinTime = Vector2(startPointX_ball, startPointY_ball) + Vector2(velocityX_ball, velocityY_ball) * minTime;
        double distance = (ourLocationAtMinTime - theirLocationAtMinTime).length();

        if (distance < dist) {
            insertCollisionData(collisionDatas, CollisionData{pathPoints[completedTimeSteps], loopTime, "BallCollision"});
            return;
        }
    }
}

void WorldObjects::calculateEnemyRobotCollisions(const rtt::world::World *world, std::vector<CollisionData> &collisionDatas, const std::vector<Vector2> &pathPoints,
                                                 double timeStep, size_t completedTimeSteps, int avoidId) {
    const std::vector<world::view::RobotView> theirRobots = world->getWorld()->getThem();
    const double maxCollisionCheckTime = 0.5;                                                                                           // Maximum time to check for collisions
    const double avoidanceDistance = 2.5 * ai::Constants::ROBOT_RADIUS_MAX() + 2 * ai::stp::control_constants::GO_TO_POS_ERROR_MARGIN;  // Distance to avoid enemy robots

    for (double loopTime = 0; loopTime < maxCollisionCheckTime; loopTime += timeStep, completedTimeSteps++) {
        if (completedTimeSteps + 1 >= pathPoints.size()) {
            return;
        }
        for (const auto &opponentRobot : theirRobots) {
            if ((pathPoints[completedTimeSteps] + (pathPoints[completedTimeSteps + 1] - pathPoints[completedTimeSteps]) / timeStep * loopTime -
                 (opponentRobot->getPos() + opponentRobot->getVel() * loopTime))
                    .length() > 1) {
                continue;
            }

            double startPointX_OurRobot = pathPoints[completedTimeSteps].x;
            double startPointY_OurRobot = pathPoints[completedTimeSteps].y;
            double startPointX_OpponentRobot = opponentRobot->getPos().x + opponentRobot->getVel().x * loopTime;
            double startPointY_OpponentRobot = opponentRobot->getPos().y + opponentRobot->getVel().y * loopTime;

            double velocityX_OurRobot = (pathPoints[completedTimeSteps + 1].x - startPointX_OurRobot) / timeStep;
            double velocityY_OurRobot = (pathPoints[completedTimeSteps + 1].y - startPointY_OurRobot) / timeStep;
            double velocityX_OpponentRobot = opponentRobot->getVel().x;
            double velocityY_OpponentRobot = opponentRobot->getVel().y;

            double velocityX_diff = velocityX_OurRobot - velocityX_OpponentRobot;
            double velocityY_diff = velocityY_OurRobot - velocityY_OpponentRobot;

            double minTime = (startPointX_OurRobot * velocityX_diff + velocityX_OurRobot * startPointX_OpponentRobot - startPointX_OpponentRobot * velocityX_OpponentRobot -
                              startPointY_OurRobot * velocityY_OurRobot + startPointY_OurRobot * velocityY_OpponentRobot + velocityY_OurRobot * startPointY_OpponentRobot -
                              startPointY_OpponentRobot * velocityY_OpponentRobot) /
                             (velocityX_OurRobot * velocityX_OurRobot - 2 * velocityX_OurRobot * velocityX_OpponentRobot + velocityX_OpponentRobot * velocityX_OpponentRobot +
                              velocityY_diff * velocityY_diff);
            if (minTime < 0) {
                minTime = 0;
            }
            if (minTime > timeStep) {
                minTime = timeStep;
            }

            // check distance at minTime
            Vector2 ourLocationAtMinTime = Vector2(startPointX_OurRobot, startPointY_OurRobot) + Vector2(velocityX_OurRobot, velocityY_OurRobot) * minTime;
            Vector2 theirLocationAtMinTime = Vector2(startPointX_OpponentRobot, startPointY_OpponentRobot) + Vector2(velocityX_OpponentRobot, velocityY_OpponentRobot) * minTime;
            double distance = (ourLocationAtMinTime - theirLocationAtMinTime).length();
            if (distance < avoidanceDistance) {
                insertCollisionData(collisionDatas, CollisionData{ourLocationAtMinTime, loopTime, "TheirRobotCollision"});
                return;
            }
        }
    }
}

void WorldObjects::calculateOurRobotCollisions(const rtt::world::World *world, std::vector<CollisionData> &collisionDatas, const std::vector<Vector2> &pathPoints,
                                               const std::unordered_map<int, std::vector<Vector2>> &computedPaths, int robotId, double timeStep, size_t completedTimeSteps) {
    auto ourRobots = world->getWorld()->getUs();
    const double maxCollisionCheckTime = 0.5;                                                                                       // Maximum time to check for collisions
    const double avoidanceDistance = 2.5 * ai::Constants::ROBOT_RADIUS() + 2 * ai::stp::control_constants::GO_TO_POS_ERROR_MARGIN;  // Distance to avoid our robots

    for (double loopTime = 0; loopTime < maxCollisionCheckTime; loopTime += timeStep, completedTimeSteps++) {
        if (completedTimeSteps + 1 >= pathPoints.size()) {
            return;
        }
        for (const auto &ourRobot : ourRobots) {
            if (ourRobot->getId() == robotId) continue;
            if ((pathPoints[completedTimeSteps] - (ourRobot->getPos() + ourRobot->getVel() * loopTime)).length() > 1) {
                continue;
            }

            double startPointX_OurRobot = pathPoints[completedTimeSteps].x;
            double startPointY_OurRobot = pathPoints[completedTimeSteps].y;
            double startPointX_OtherRobot = ourRobot->getPos().x + ourRobot->getVel().x * loopTime;
            double startPointY_OtherRobot = ourRobot->getPos().y + ourRobot->getVel().y * loopTime;

            double velocityX_OurRobot = (pathPoints[completedTimeSteps + 1].x - startPointX_OurRobot) / timeStep;
            double velocityY_OurRobot = (pathPoints[completedTimeSteps + 1].y - startPointY_OurRobot) / timeStep;
            double velocityX_OtherRobot = ourRobot->getVel().x;
            double velocityY_OtherRobot = ourRobot->getVel().y;

            double velocityX_diff = velocityX_OurRobot - velocityX_OtherRobot;
            double velocityY_diff = velocityY_OurRobot - velocityY_OtherRobot;

            double minTime = (startPointX_OurRobot * velocityX_diff + velocityX_OurRobot * startPointX_OtherRobot - startPointX_OtherRobot * velocityX_OtherRobot -
                              startPointY_OurRobot * velocityY_OurRobot + startPointY_OurRobot * velocityY_OtherRobot + velocityY_OurRobot * startPointY_OtherRobot -
                              startPointY_OtherRobot * velocityY_OtherRobot) /
                             (velocityX_OurRobot * velocityX_OurRobot - 2 * velocityX_OurRobot * velocityX_OtherRobot + velocityX_OtherRobot * velocityX_OtherRobot +
                              velocityY_diff * velocityY_diff);
            if (minTime < 0) {
                minTime = 0;
            }
            if (minTime > timeStep) {
                minTime = timeStep;
            }

            Vector2 ourLocationAtMinTime = Vector2(startPointX_OurRobot, startPointY_OurRobot) + Vector2(velocityX_OurRobot, velocityY_OurRobot) * minTime;
            Vector2 otherRobotLocationAtMinTime = Vector2(startPointX_OtherRobot, startPointY_OtherRobot) + Vector2(velocityX_OtherRobot, velocityY_OtherRobot) * minTime;
            double distance = (ourLocationAtMinTime - otherRobotLocationAtMinTime).length();
            if (distance < avoidanceDistance) {
                insertCollisionData(collisionDatas, CollisionData{ourLocationAtMinTime, loopTime, "OurRobotCollision"});
                return;
            }
        }
    }
}

void WorldObjects::calculateBallPlacementCollision(const rtt::world::World *world, std::vector<CollisionData> &collisionDatas, const std::vector<Vector2> &pathPoints,
                                                   double timeStep, size_t completedTimeSteps) {
    auto ballPlacementPos = rtt::ai::GameStateManager::getRefereeDesignatedPosition();
    auto startPositionBall = world->getWorld()->getBall()->get()->position;
    // line segment from startPosition to ballPlacementPos
    LineSegment ballPlacementLine(startPositionBall, ballPlacementPos);
    double time = completedTimeSteps * timeStep;

    for (size_t i = completedTimeSteps; i < pathPoints.size(); i++) {
        if (ballPlacementLine.distanceToLine(pathPoints[i]) < ai::stp::control_constants::AVOID_BALL_DISTANCE) {
            insertCollisionData(collisionDatas, CollisionData{pathPoints[i], timeStep * (i - completedTimeSteps), "BallPlacementCollision"});
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
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
    if (avoidObjects.shouldAvoidTheirRobots) {
        calculateEnemyRobotCollisions(world, collisionDatas, pathPoints, timeStep, completedTimeSteps);
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

            insertCollisionData(collisionDatas, CollisionData{pathPoints[i], i * timeStep, "FieldCollision"});
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

            insertCollisionData(collisionDatas, CollisionData{pathPoints[i], i * timeStep, "DefenseAreaCollision"});
            return;
        }
    }
}

void WorldObjects::calculateBallCollisions(const rtt::world::World *world, std::vector<CollisionData> &collisionDatas, std::vector<Vector2> pathPoints, double timeStep,
                                           double dist, size_t completedTimeSteps) {
    auto startPositionBall = world->getWorld()->getBall()->get()->position;
    auto VelocityBall = world->getWorld()->getBall()->get()->velocity;
    std::vector<Vector2> ballTrajectory;

    double time = completedTimeSteps * timeStep;
    double ballAvoidanceTime = 2;

    while (pathPoints.size() * timeStep > time && time < ballAvoidanceTime - completedTimeSteps * timeStep) {
        ballTrajectory.emplace_back(startPositionBall + VelocityBall * time);
        time += timeStep;
    }

    for (size_t i = completedTimeSteps; i < ballTrajectory.size(); i++) {
        if (pathPoints[i].dist(ballTrajectory[i]) < dist) {
            insertCollisionData(collisionDatas, CollisionData{pathPoints[i], i * timeStep, "BallCollision"});
            return;
        }
    }
}

void WorldObjects::calculateEnemyRobotCollisions(const rtt::world::World *world, std::vector<CollisionData> &collisionDatas, const std::vector<Vector2> &pathPoints,
                                                 double timeStep, size_t completedTimeSteps) {
    const std::vector<world::view::RobotView> theirRobots = world->getWorld()->getThem();
    const double maxCollisionCheckTime = 1.0;                                  // Maximum time to check for collisions
    const double avoidanceDistance = 2.5 * ai::Constants::ROBOT_RADIUS_MAX();  // Distance to avoid enemy robots

    for (size_t i = completedTimeSteps + 1; i < pathPoints.size() - 1; i++) {
        double currentTime = i * timeStep;
        if (currentTime - completedTimeSteps * timeStep >= maxCollisionCheckTime) break;

        // Interpolate between the current point and the next point
        Vector2 startPoint = pathPoints[i];
        Vector2 endPoint = pathPoints[i + 1];
        for (double t = 0; t <= 1; t += 0.3) {
            Vector2 interpolatedPoint = startPoint * (1 - t) + endPoint * t;
            double interpolatedTime = currentTime + t * timeStep;

            for (const auto &theirRobot : theirRobots) {
                Vector2 theirVel = theirRobot->getVel();
                Vector2 theirPos = theirRobot->getPos() + theirVel * interpolatedTime;
                Vector2 posDif = interpolatedPoint - theirPos;

                if (posDif.length() < avoidanceDistance) {
                    insertCollisionData(collisionDatas, CollisionData{interpolatedPoint, interpolatedTime, "EnemyRobotCollision"});
                    return;
                }
            }
        }
    }
}
void WorldObjects::calculateOurRobotCollisions(const rtt::world::World *world, std::vector<CollisionData> &collisionDatas, const std::vector<Vector2> &pathPoints,
                                               const std::unordered_map<int, std::vector<Vector2>> &computedPaths, int robotId, double timeStep, size_t completedTimeSteps) {
    auto ourRobots = world->getWorld()->getUs();
    const double maxCollisionCheckTime = 1.0;                              // Maximum time to check for collisions
    const double avoidanceDistance = 2.0 * ai::Constants::ROBOT_RADIUS();  // Distance to avoid our robots
    // Because wallers are irritating and colide with each other, we ignore first 10 timesteps
    // Otherwise, we get collisions in every direction and we just yoink through our own defense are
    // TODO: Maybe fix this in a better way if we get a lot of collisions.
    for (size_t i = completedTimeSteps + 6; i < pathPoints.size() - 1; i++) {
        double currentTime = i * timeStep;
        if (currentTime - completedTimeSteps * timeStep >= maxCollisionCheckTime) break;

        // Interpolate between the current point and the next point
        Vector2 startPoint = pathPoints[i];
        Vector2 endPoint = pathPoints[i + 1];
        for (double t = 0; t <= 1; t += 0.3) {
            Vector2 interpolatedPoint = startPoint * (1 - t) + endPoint * t;
            double interpolatedTime = currentTime + t * timeStep;

            for (auto &robot : ourRobots) {
                if (robotId != robot->getId()) {
                    Vector2 theirPos;
                    if (computedPaths.find(robot->getId()) != computedPaths.end() && computedPaths.at(robot->getId()).size() > (i + completedTimeSteps)) {
                        theirPos = computedPaths.at(robot->getId())[i + completedTimeSteps] * (1 - t) + computedPaths.at(robot->getId())[i + completedTimeSteps + 1] * t;
                    } else {
                        theirPos = robot->getPos();
                    }

                    if ((interpolatedPoint - theirPos).length() < avoidanceDistance) {
                        insertCollisionData(collisionDatas, CollisionData{interpolatedPoint, interpolatedTime, "OurRobotCollision"});
                        return;
                    }
                }
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
        if (ballPlacementLine.distanceToLine(pathPoints[i]) < 0.4) {
            insertCollisionData(collisionDatas, CollisionData{pathPoints[i], i * timeStep, "BallPlacementCollision"});
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
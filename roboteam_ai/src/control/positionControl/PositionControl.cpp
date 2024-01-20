//
// Created by ratoone on 18-11-19.
//

#include "control/positionControl/PositionControl.h"

#include "control/positionControl/BBTrajectories/BBTrajectory2D.h"
#include "gui/Out.h"
#include "roboteam_utils/Print.h"
#include "world/World.hpp"

namespace rtt::ai::control {
std::vector<Vector2> PositionControl::getComputedPath(int ID) { return computedPaths[ID]; }

RobotCommand PositionControl::computeAndTrackPath(const rtt::Field &field, int robotId, const Vector2 &currentPosition, const Vector2 &currentVelocity, Vector2 &targetPosition,
                                                  stp::PIDType pidType) {
    collisionDetector.setField(field);

    // if the robot is close to the final position and can't get there, stop
    if ((currentPosition - targetPosition).length() < FINAL_AVOIDANCE_DISTANCE && collisionDetector.getRobotCollisionBetweenPoints(currentPosition, targetPosition)) {
        RTT_INFO("Path collides with something close to the target position for robot ID ", robotId)
        return {};
    }
    if (shouldRecalculatePath(currentPosition, targetPosition, currentVelocity, robotId)) {
        computedPaths[robotId] = pathPlanningAlgorithm.computePath(currentPosition, targetPosition);
    }

    RobotCommand command = {};
    Position trackingVelocity = pathTrackingAlgorithm.trackPathDefaultAngle(currentPosition, currentVelocity, computedPaths[robotId], robotId, pidType);
    command.velocity = Vector2(trackingVelocity.x, trackingVelocity.y);
    command.targetAngle = trackingVelocity.rot;
    return command;
}

bool PositionControl::shouldRecalculatePath(const Vector2 &currentPosition, const Vector2 &targetPos, const Vector2 &currentVelocity, int robotId) {
    return computedPaths[robotId].empty() || PositionControlUtils::isTargetChanged(targetPos, computedPaths[robotId].back()) ||
           (currentVelocity != Vector2(0, 0) && collisionDetector.isCollisionBetweenPoints(currentPosition, computedPaths[robotId].front()));
}

void PositionControl::setRobotPositions(std::vector<Vector2> &robotPositions) { collisionDetector.setRobotPositions(robotPositions); }

rtt::BB::CommandCollision PositionControl::computeAndTrackTrajectory(const rtt::world::World *world, const rtt::Field &field, int robotId, Vector2 currentPosition,
                                                                     Vector2 currentVelocity, Vector2 targetPosition, double maxRobotVelocity, stp::PIDType pidType,
                                                                     stp::AvoidObjects avoidObjects) {
    double timeStep = 0.1;

    std::optional<BB::CollisionData> firstCollision;
    rtt::BB::CommandCollision commandCollision;

    if (shouldRecalculateTrajectory(world, field, robotId, targetPosition, currentPosition, avoidObjects)) {
        computedTrajectories[robotId] = Trajectory2D(currentPosition, currentVelocity, targetPosition, maxRobotVelocity, ai::Constants::MAX_ACC_UPPER());
        completedTimeSteps[robotId] = 0;
        firstCollision = worldObjects.getFirstCollision(world, field, computedTrajectories[robotId], computedPaths, robotId, avoidObjects, completedTimeSteps);
        if (firstCollision.has_value()) {
            auto newTrajectory =
                findNewTrajectory(world, field, robotId, currentPosition, currentVelocity, firstCollision, targetPosition, maxRobotVelocity, timeStep, avoidObjects);
            if (newTrajectory.has_value()) {
                computedTrajectories[robotId] = newTrajectory.value();
            } else {
                commandCollision.collisionData = firstCollision;
            }
        }

        computedPaths[robotId] = computedTrajectories[robotId].getPathApproach(timeStep);
        computedPathsVel[robotId] = computedTrajectories[robotId].getVelocityVector(timeStep);
        computedPathsPosVel[robotId].clear();
        computedPathsPosVel[robotId].reserve(computedPaths[robotId].size());
        for (size_t i = 0; i < computedPaths[robotId].size(); i++) {
            computedPathsPosVel[robotId].push_back(std::make_pair(computedPaths[robotId][i], computedPathsVel[robotId][i]));
        }
    }

    rtt::ai::gui::Out::draw(
        {
            .label = "path_lines" + std::to_string(robotId),
            .color = proto::Drawing::MAGENTA,
            .method = proto::Drawing::LINES_CONNECTED,
            .category = proto::Drawing::PATH_PLANNING,
            .forRobotId = robotId,
            .thickness = 1,
        },
        computedPaths[robotId]);

    rtt::ai::gui::Out::draw(
        {
            .label = "path_dots" + std::to_string(robotId),
            .color = proto::Drawing::GREEN,
            .method = proto::Drawing::DOTS,
            .category = proto::Drawing::PATH_PLANNING,
            .forRobotId = robotId,
            .size = 2,
        },
        computedPaths[robotId]);

    // If you are closer to the target than the first point of the approximated path, remove it
    if (computedPaths[robotId].size() > 1 && (targetPosition - currentPosition).length() < (targetPosition - computedPaths[robotId].front()).length()) {
        computedPaths[robotId].erase(computedPaths[robotId].begin());
        completedTimeSteps[robotId]++;
    }

    commandCollision.robotCommand = {};
    // Position trackingVelocity = pathTrackingAlgorithm.trackPathDefaultAngle(currentPosition, currentVelocity,computedPaths[robotId], robotId, pidType);
    Position trackingVelocity = pathTrackingAlgorithmBBT.trackPathForwardAngle(currentPosition, currentVelocity, computedPathsPosVel[robotId], robotId, pidType);
    Vector2 trackingVelocityVector = {trackingVelocity.x, trackingVelocity.y};

    // If there is a collision on the path (so no collision-free path could be found), lower the speed to 1 m/s
    if (commandCollision.collisionData.has_value()) {
        if (trackingVelocityVector.length() > 1) trackingVelocityVector = trackingVelocityVector.stretchToLength(1);
    }

    commandCollision.robotCommand.velocity = trackingVelocityVector;
    commandCollision.robotCommand.targetAngle = trackingVelocity.rot;

    return commandCollision;
}

double PositionControl::calculateScore(const rtt::world::World *world, const rtt::Field &field, int robotId, std::optional<BB::CollisionData> &firstCollision,
                                       Trajectory2D &trajectoryAroundCollision, stp::AvoidObjects avoidObjects) {
    double totalTime = trajectoryAroundCollision.getTotalTime();
    double score = totalTime;
    if (!firstCollision.has_value()) {
        return score;
    }

    score += 5;
    double timeTillFirstCollision = firstCollision.value().collisionTime;
    score += std::max(0.0, 3 - timeTillFirstCollision);

    if (avoidObjects.shouldAvoidDefenseArea) {
        auto defenseAreaCollision = worldObjects.getFirstDefenseAreaCollision(field, trajectoryAroundCollision, computedPaths, robotId);
        if (defenseAreaCollision.has_value()) {
            score += std::max(0.0, 1 - defenseAreaCollision.value().collisionTime) * 10;
            score += 5;
        }
    }

    auto currentStrategyName = rtt::ai::GameStateManager::getCurrentGameState().getStrategyName();
    if (currentStrategyName == "ball_placement_them") {
        auto ballPlacementPos = rtt::ai::GameStateManager::getRefereeDesignatedPosition();
        auto startPositionBall = world->getWorld()->getBall()->get()->position;
        Line ballPlacementLine(startPositionBall, ballPlacementPos);
        Vector2 p1 = firstCollision.value().collisionPosition;
        Vector2 p2 = trajectoryAroundCollision.getPosition(totalTime);
        if (ballPlacementLine.arePointsOnOppositeSides(p1, p2)) {
            double d1 = (p1 - ballPlacementPos).length() + (p2 - ballPlacementPos).length();
            double d2 = (p1 - startPositionBall).length() + (p2 - startPositionBall).length();
            score += std::min(d1, d2) * 10;
            score += 10;
        }
    }

    return score;
}

std::optional<Trajectory2D> PositionControl::findNewTrajectory(const rtt::world::World *world, const rtt::Field &field, int robotId, Vector2 &currentPosition,
                                                               Vector2 &currentVelocity, std::optional<BB::CollisionData> &firstCollision, Vector2 &targetPosition,
                                                               double maxRobotVelocity, double timeStep, stp::AvoidObjects avoidObjects) {
    auto intermediatePoints = createIntermediatePoints(field, firstCollision, targetPosition);
    // order intermediatepoints such that points closer to the target are first
    std::sort(intermediatePoints.begin(), intermediatePoints.end(),
              [&targetPosition](const Vector2 &a, const Vector2 &b) { return (a - targetPosition).length() < (b - targetPosition).length(); });

    double bestScore = 999;
    std::optional<Trajectory2D> bestTrajectory = std::nullopt;

    for (const auto &intermediatePoint : intermediatePoints) {
        Trajectory2D trajectoryToIntermediatePoint(currentPosition, currentVelocity, intermediatePoint, maxRobotVelocity, ai::Constants::MAX_ACC_UPPER());

        auto intermediatePathCollision = worldObjects.getFirstCollision(world, field, trajectoryToIntermediatePoint, computedPaths, robotId, avoidObjects, completedTimeSteps);
        auto trajectoryAroundCollision = calculateTrajectoryAroundCollision(world, field, intermediatePathCollision, trajectoryToIntermediatePoint, targetPosition, robotId,
                                                                            maxRobotVelocity, timeStep, avoidObjects);
        if (trajectoryAroundCollision.has_value()) {
            auto firstCollision = worldObjects.getFirstCollision(world, field, trajectoryAroundCollision.value(), computedPaths, robotId, avoidObjects, completedTimeSteps);
            double score = calculateScore(world, field, robotId, firstCollision, trajectoryAroundCollision.value(), avoidObjects);
            if (score < bestScore) {
                bestScore = score;
                bestTrajectory = trajectoryAroundCollision.value();
            }
        }
    }
    return bestTrajectory;
}

std::optional<Trajectory2D> PositionControl::calculateTrajectoryAroundCollision(const rtt::world::World *world, const rtt::Field &field,
                                                                                std::optional<BB::CollisionData> &intermediatePathCollision,
                                                                                Trajectory2D trajectoryToIntermediatePoint, Vector2 &targetPosition, int robotId,
                                                                                double maxRobotVelocity, double timeStep, stp::AvoidObjects avoidObjects) {
    double bestScore = 999;
    double bestTime = 0;
    std::optional<Trajectory2D> bestTrajectory = std::nullopt;
    std::optional<Trajectory2D> bestIntermediateToTarget = std::nullopt;

    timeStep *= 2;
    int numberOfTimeSteps = floor(trajectoryToIntermediatePoint.getTotalTime() / timeStep);
    for (int i = 0; i < numberOfTimeSteps; i++) {
        Vector2 newStart = trajectoryToIntermediatePoint.getPosition(i * timeStep);
        Vector2 newVelocity = trajectoryToIntermediatePoint.getVelocity(i * timeStep);

        Trajectory2D intermediateToTarget(newStart, newVelocity, targetPosition, maxRobotVelocity, ai::Constants::MAX_ACC_UPPER());

        auto newStartCollisions = worldObjects.getFirstCollision(world, field, intermediateToTarget, computedPaths, robotId, avoidObjects, completedTimeSteps);

        if (newStartCollisions.has_value()) {
            double totalTime = i * timeStep + intermediateToTarget.getTotalTime();
            double timeTillFirstCollision = newStartCollisions.value().collisionTime;
            double score = totalTime + 5 + std::max(0.0, 3 - timeTillFirstCollision);
            if (avoidObjects.shouldAvoidDefenseArea) {
                auto defenseAreaCollision = worldObjects.getFirstDefenseAreaCollision(field, intermediateToTarget, computedPaths, robotId);
                if (defenseAreaCollision.has_value()) {
                    score += std::max(0.0, 1 - defenseAreaCollision.value().collisionTime);
                }
            }

            if (rtt::ai::GameStateManager::getCurrentGameState().getStrategyName() == "ball_placement_them") {
                auto ballPlacementPos = rtt::ai::GameStateManager::getRefereeDesignatedPosition();
                auto startPositionBall = world->getWorld()->getBall()->get()->position;
                Line ballPlacementLine(startPositionBall, ballPlacementPos);
                Vector2 p1 = newStartCollisions.value().collisionPosition;
                Vector2 p2 = targetPosition;
                if (ballPlacementLine.arePointsOnOppositeSides(p1, p2)) {
                    double d1 = (p1 - ballPlacementPos).length() + (p2 - ballPlacementPos).length();
                    double d2 = (p1 - startPositionBall).length() + (p2 - startPositionBall).length();
                    d2 += 999;
                    score += std::min(d1, d2) * 10;
                    score += 10;
                }
            }

            if (score < bestScore) {
                bestScore = score;
                bestTrajectory = trajectoryToIntermediatePoint;
                bestIntermediateToTarget = intermediateToTarget;
                bestTime = i * timeStep;
            }
        } else {
            trajectoryToIntermediatePoint.addTrajectory(intermediateToTarget, i * timeStep);
            trajectoryToIntermediatePoint.getTotalTime();
            return trajectoryToIntermediatePoint;  // This is now the whole path
        }
    }
    // If there was a collision in all intermediate path, return the best free trajectory
    if (bestTrajectory.has_value()) {
        bestTrajectory.value().addTrajectory(bestIntermediateToTarget.value(), bestTime);
    }
    return bestTrajectory;
}

std::vector<Vector2> PositionControl::createIntermediatePoints(const rtt::Field &field, std::optional<BB::CollisionData> &firstCollision, Vector2 &targetPosition) {
    float angleBetweenIntermediatePoints = M_PI_4 / 2;
    float fieldHeight = field.playArea.height();
    float pointExtension = fieldHeight / 18;
    Vector2 collisionToTargetNormalized = (firstCollision->collisionPosition - targetPosition).normalize();
    Vector2 pointToDrawFrom = firstCollision->collisionPosition + collisionToTargetNormalized * pointExtension;

    std::vector<Vector2> intermediatePoints;
    float intermediatePointRadius = fieldHeight / 4;
    Vector2 pointToRotate = pointToDrawFrom + collisionToTargetNormalized * intermediatePointRadius;
    for (int i = -9; i < 9; i++) {
        if (i != 0) {
            Vector2 intermediatePoint = pointToRotate.rotateAroundPoint(i * angleBetweenIntermediatePoints, pointToDrawFrom);

            if (field.playArea.contains(intermediatePoint)) {
                intermediatePoints.emplace_back(intermediatePoint);
            }
        }
    }
    return intermediatePoints;
}

bool PositionControl::shouldRecalculateTrajectory(const rtt::world::World *world, const rtt::Field &field, int robotId, Vector2 targetPosition, const Vector2 &currentPosition,
                                                  ai::stp::AvoidObjects avoidObjects) {
    if (!computedTrajectories.contains(robotId) ||
        (computedPaths.contains(robotId) && !computedPaths[robotId].empty() &&
         (targetPosition - computedPaths[robotId][computedPaths[robotId].size() - 1]).length() > stp::control_constants::GO_TO_POS_ERROR_MARGIN) ||
        worldObjects.getFirstCollision(world, field, computedTrajectories[robotId], computedPaths, robotId, avoidObjects, completedTimeSteps).has_value()) {
        return true;
    }

    // FIXME: Bankonk hotfix. Mere the new BBT from PR.
    // If next point on path is too far away from the robot
    if ((computedPaths[robotId][0] - currentPosition).length() > 0.5) {
        return true;
    }

    return false;
}

}  // namespace rtt::ai::control
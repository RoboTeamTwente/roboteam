//
// Created by ratoone on 18-11-19.
//

#include "control/positionControl/PositionControl.h"

#include "control/positionControl/BBTrajectories/BBTrajectory2D.h"
#include "gui/Out.h"
#include "roboteam_utils/Print.h"

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

        // Check path to original target for collisions
        firstCollision = worldObjects.getFirstCollision(world, field, computedTrajectories[robotId], computedPaths, robotId, avoidObjects);

        if (firstCollision.has_value()) {
            //            RTT_DEBUG(firstCollision->collisionName);
            // Create intermediate points, return a collision-free trajectory originating from the best option of these points
            auto newTrajectory =
                findNewTrajectory(world, field, robotId, currentPosition, currentVelocity, firstCollision, targetPosition, maxRobotVelocity, timeStep, avoidObjects);
            if (newTrajectory.has_value()) {
                computedTrajectories[robotId] = newTrajectory.value();
            } else {
                commandCollision.collisionData = firstCollision;
                // RTT_DEBUG("Could not find a collision-free path");
            }
        }

        computedPaths[robotId] = computedTrajectories[robotId].getPathApproach(timeStep);
        computedPathsVel[robotId] = computedTrajectories[robotId].getVelocityVector(timeStep);  // creates a vector with all the velocities
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

    // Current method is very hacky
    // If you are closer to the target than the first point of the approximated path, remove it
    if (computedPaths[robotId].size() > 1 && (targetPosition - currentPosition).length() < (targetPosition - computedPaths[robotId].front()).length()) {
        computedPaths[robotId].erase(computedPaths[robotId].begin());
    }

    commandCollision.robotCommand = {};
    // Position trackingVelocity = pathTrackingAlgorithm.trackPathDefaultAngle(currentPosition, currentVelocity,computedPaths[robotId], robotId, pidType);
    Position trackingVelocity = pathTrackingAlgorithmBBT.trackPathForwardAngle(currentPosition, currentVelocity, computedPathsPosVel[robotId], robotId, pidType);
    Vector2 trackingVelocityVector = {trackingVelocity.x, trackingVelocity.y};

    // If there is a collision on the path (so no collision-free path could be found), lower the speed to 1 m/s. This increases the chances of finding a new path
    // while also decreasing the speed at which collisions happen
    if (commandCollision.collisionData.has_value()) {
        if (trackingVelocityVector.length() > 1) trackingVelocityVector = trackingVelocityVector.stretchToLength(1);
    }

    commandCollision.robotCommand.velocity = trackingVelocityVector;
    commandCollision.robotCommand.targetAngle = trackingVelocity.rot;

    return commandCollision;
}
std::optional<Trajectory2D> PositionControl::findNewTrajectory(const rtt::world::World *world, const rtt::Field &field, int robotId, Vector2 &currentPosition,
                                                               Vector2 &currentVelocity, std::optional<BB::CollisionData> &firstCollision, Vector2 &targetPosition,
                                                               double maxRobotVelocity, double timeStep, stp::AvoidObjects avoidObjects) {
    auto intermediatePoints = createIntermediatePoints(field, robotId, firstCollision, targetPosition);
    auto intermediatePointsSorted = scoreIntermediatePoints(intermediatePoints, firstCollision);

    double longestTimeTillCollision = 0;
    std::optional<Trajectory2D> bestTrajectory = std::nullopt;

    while (!intermediatePointsSorted.empty()) {
        Trajectory2D trajectoryToIntermediatePoint(currentPosition, currentVelocity, intermediatePointsSorted.top().second, maxRobotVelocity, ai::Constants::MAX_ACC_UPPER());

        auto intermediatePathCollision = worldObjects.getFirstCollision(world, field, trajectoryToIntermediatePoint, computedPaths, robotId, avoidObjects);
        auto trajectoryAroundCollision = calculateTrajectoryAroundCollision(world, field, intermediatePathCollision, trajectoryToIntermediatePoint, targetPosition, robotId,
                                                                            maxRobotVelocity, timeStep, avoidObjects);
        if (trajectoryAroundCollision.has_value()) {
            auto firstCollision = worldObjects.getFirstCollision(world, field, trajectoryAroundCollision.value(), computedPaths, robotId, avoidObjects);
            double timeTillCollision = firstCollision.has_value() ? firstCollision.value().collisionTime : trajectoryAroundCollision.value().getTotalTime();
            if (timeTillCollision > longestTimeTillCollision) {
                longestTimeTillCollision = timeTillCollision;
                bestTrajectory = trajectoryAroundCollision.value();
            }
        }
        intermediatePointsSorted.pop();
    }   
    return bestTrajectory;
}

std::optional<Trajectory2D> PositionControl::calculateTrajectoryAroundCollision(const rtt::world::World *world, const rtt::Field &field,
                                                                                std::optional<BB::CollisionData> &intermediatePathCollision,
                                                                                Trajectory2D trajectoryToIntermediatePoint, Vector2 &targetPosition, int robotId,
                                                                                double maxRobotVelocity, double timeStep, stp::AvoidObjects avoidObjects) {
    double longestTimeTillCollision = 0;
    std::optional<Trajectory2D> longestTrajectory = std::nullopt;
    std::optional<Trajectory2D> longestIntermediateToTarget = std::nullopt;
    double longestTime = 0;

    if (!intermediatePathCollision.has_value()) {
        timeStep *= 2;
        int numberOfTimeSteps = floor(trajectoryToIntermediatePoint.getTotalTime() / timeStep);
        for (int i = 0; i < numberOfTimeSteps; i++) {
            Vector2 newStart = trajectoryToIntermediatePoint.getPosition(i * timeStep);
            Vector2 newVelocity = trajectoryToIntermediatePoint.getVelocity(i * timeStep);

            Trajectory2D intermediateToTarget(newStart, newVelocity, targetPosition, maxRobotVelocity, ai::Constants::MAX_ACC_UPPER());

            auto newStartCollisions = worldObjects.getFirstCollision(world, field, intermediateToTarget, computedPaths, robotId, avoidObjects);

            if (newStartCollisions.has_value()) {
                double timeTillCollision = newStartCollisions.value().collisionTime;

                if (timeTillCollision > longestTimeTillCollision) {
                    longestTimeTillCollision = timeTillCollision;
                    longestTrajectory = trajectoryToIntermediatePoint;
                    longestIntermediateToTarget = intermediateToTarget;
                    longestTime = i * timeStep;
                }
            } else {
                trajectoryToIntermediatePoint.addTrajectory(intermediateToTarget, i * timeStep);
                return trajectoryToIntermediatePoint;  // This is now the whole path
            }
        }
    }
    // If there was a collision in all intermediate path, return the longest collision free trajectory
    if (longestTrajectory.has_value()) {
        longestTrajectory.value().addTrajectory(longestIntermediateToTarget.value(), longestTime);
    }
    return longestTrajectory;
}

std::vector<Vector2> PositionControl::createIntermediatePoints(const rtt::Field &field, int robotId, std::optional<BB::CollisionData> &firstCollision, Vector2 &targetPosition) {
    double angleBetweenIntermediatePoints = M_PI_4 / 2;

    // Radius and point extension of intermediate points are based on the fieldHeight
    auto fieldHeight = field.playArea.height();

    // PointToDrawFrom is picked by drawing a line from the target position to the obstacle and extending that
    // line further towards our currentPosition by extension meters.
    float pointExtension = fieldHeight / 18;  // How far the pointToDrawFrom has to be from the obstaclePosition
    Vector2 pointToDrawFrom = firstCollision->obstaclePosition + (firstCollision->obstaclePosition - targetPosition).normalize() * pointExtension;

    std::vector<Vector2> intermediatePoints;
    for (int i = -4; i < 5; i++) {
        if (i != 0) {
            // Make half circle of intermediatePoints pointed towards obstaclePosition, originating from pointToDrawFrom, by rotating pointToRotate with a radius
            // intermediatePointRadius
            float intermediatePointRadius = fieldHeight / 4;  // Radius of the half circle
            Vector2 pointToRotate = pointToDrawFrom + (targetPosition - firstCollision->obstaclePosition).normalize() * intermediatePointRadius;
            Vector2 intermediatePoint = pointToRotate.rotateAroundPoint(i * angleBetweenIntermediatePoints, pointToDrawFrom);

            if (!field.playArea.contains(intermediatePoint)) {
                continue;
            }
            intermediatePoints.emplace_back(intermediatePoint);
        }
    }
    return intermediatePoints;
}

std::priority_queue<std::pair<double, Vector2>, std::vector<std::pair<double, Vector2>>, std::greater<>> PositionControl::scoreIntermediatePoints(
    std::vector<Vector2> &intermediatePoints, std::optional<BB::CollisionData> &firstCollision) {
    double intermediatePointScore;
    std::priority_queue<std::pair<double, Vector2>, std::vector<std::pair<double, Vector2>>, std::greater<>> intermediatePointsSorted;
    for (const auto &i : intermediatePoints) {
        intermediatePointScore = (i - firstCollision->collisionPosition).length();
        std::pair<double, Vector2> p = {intermediatePointScore, i};
        intermediatePointsSorted.push(p);
    }
    return intermediatePointsSorted;
}

bool PositionControl::shouldRecalculateTrajectory(const rtt::world::World *world, const rtt::Field &field, int robotId, Vector2 targetPosition, const Vector2 &currentPosition,
                                                  ai::stp::AvoidObjects avoidObjects) {
    if (!computedTrajectories.contains(robotId) ||
        (computedPaths.contains(robotId) && !computedPaths[robotId].empty() &&
         (targetPosition - computedPaths[robotId][computedPaths[robotId].size() - 1]).length() > stp::control_constants::GO_TO_POS_ERROR_MARGIN) ||
        worldObjects.getFirstCollision(world, field, computedTrajectories[robotId], computedPaths, robotId, avoidObjects).has_value()) {
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
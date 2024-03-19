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
    std::optional<BB::CollisionData> firstCollision;
    rtt::BB::CommandCollision commandCollision = {};

    if (shouldRecalculateTrajectory(world, field, robotId, targetPosition, currentPosition, avoidObjects)) {
        double timeStep = 0.1;
        computedTrajectories[robotId] = Trajectory2D(currentPosition, currentVelocity, targetPosition, maxRobotVelocity, ai::Constants::MAX_ACC_UPPER());
        completedTimeSteps[robotId] = 0;
        firstCollision = worldObjects.getFirstCollision(world, field, computedTrajectories[robotId], computedPaths, robotId, avoidObjects, completedTimeSteps);
        if (firstCollision.has_value()) {
            auto newTrajectory =
                findNewTrajectory(world, field, robotId, currentPosition, currentVelocity, firstCollision, targetPosition, maxRobotVelocity, timeStep, avoidObjects);
            if (newTrajectory.has_value()) {
                computedTrajectories[robotId] = newTrajectory.value();
            }
            firstCollision = worldObjects.getFirstCollision(world, field, computedTrajectories[robotId], computedPaths, robotId, avoidObjects, completedTimeSteps);
            if (firstCollision.has_value()) {
                commandCollision.collisionData = firstCollision;

                if ((firstCollision.value().collisionType == BB::CollisionType::BALL) && firstCollision.value().collisionTime <= 0.11) {
                    targetPosition = handleBallCollision(world, field, currentPosition, avoidObjects);
                    computedTrajectories[robotId] = Trajectory2D(currentPosition, currentVelocity, targetPosition, 1.0, ai::Constants::MAX_ACC_UPPER());
                }
                if ((firstCollision.value().collisionType == BB::CollisionType::BALLPLACEMENT) && firstCollision.value().collisionTime <= 0.11) {
                    targetPosition = handleBallPlacementCollision(world, field, currentPosition, avoidObjects);
                    computedTrajectories[robotId] = Trajectory2D(currentPosition, currentVelocity, targetPosition, 1.0, ai::Constants::MAX_ACC_UPPER());
                }
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

    // Position trackingVelocity = pathTrackingAlgorithm.trackPathDefaultAngle(currentPosition, currentVelocity,computedPaths[robotId], robotId, pidType);
    Position trackingVelocity = pathTrackingAlgorithmBBT.trackPathForwardAngle(currentPosition, currentVelocity, computedPathsPosVel[robotId], robotId, pidType);
    Vector2 trackingVelocityVector = {trackingVelocity.x, trackingVelocity.y};

    // Break if all paths result in collision and we will collide with a robot
    if (commandCollision.collisionData.has_value() && (commandCollision.collisionData.value().collisionType == BB::CollisionType::ENEMYROBOT ||
                                                       commandCollision.collisionData.value().collisionType == BB::CollisionType::OWNROBOT)) {
        if (trackingVelocityVector.length() > 0.1) trackingVelocityVector = trackingVelocityVector.stretchToLength(0.1);
    }

    commandCollision.robotCommand.velocity = trackingVelocityVector;
    commandCollision.robotCommand.targetAngle = trackingVelocity.rot;

    return commandCollision;
}

rtt::Vector2 PositionControl::handleBallCollision(const rtt::world::World *world, const rtt::Field &field, Vector2 currentPosition, stp::AvoidObjects avoidObjects) {
    auto ballPos = world->getWorld()->getBall()->get()->position;
    auto direction = currentPosition - ballPos;
    Vector2 targetPosition = currentPosition + direction.stretchToLength(stp::control_constants::AVOID_BALL_DISTANCE * 2);
    if (FieldComputations::pointIsValidPosition(field, targetPosition, avoidObjects, stp::control_constants::OUT_OF_FIELD_MARGIN)) {
        return targetPosition;
    }
    int rotationStepDegrees = 10;
    int maxRotationDegrees = 90;
    for (int i = rotationStepDegrees; i <= maxRotationDegrees; i += rotationStepDegrees) {
        for (int sign : {1, -1}) {
            double rotation = sign * i * M_PI / 180;
            Vector2 rotatedDirection = direction.stretchToLength(stp::control_constants::AVOID_BALL_DISTANCE * 2).rotate(rotation);
            Vector2 potentialTargetPosition = currentPosition + rotatedDirection;
            if (FieldComputations::pointIsValidPosition(field, potentialTargetPosition, avoidObjects, stp::control_constants::OUT_OF_FIELD_MARGIN)) {
                return potentialTargetPosition;
            }
        }
    }
    return currentPosition + direction.stretchToLength(stp::control_constants::AVOID_BALL_DISTANCE * 2).rotate(M_PI / 2);
}

rtt::Vector2 PositionControl::handleBallPlacementCollision(const rtt::world::World *world, const rtt::Field &field, Vector2 currentPosition, stp::AvoidObjects avoidObjects) {
    auto placementPos = rtt::ai::GameStateManager::getRefereeDesignatedPosition();
    auto ballPos = world->getWorld()->getBall()->get()->position;
    auto direction = (placementPos - ballPos).stretchToLength(stp::control_constants::AVOID_BALL_DISTANCE * 2);
    double distance = (placementPos - ballPos).length();
    if (distance > 0.10) {  // distance is more than 10cm
        direction = direction.rotate((currentPosition - ballPos).cross(placementPos - ballPos) < 0 ? M_PI / 2 : -M_PI / 2);
    } else {  // distance is less than or equal to 10cm
        direction = (currentPosition - ballPos).stretchToLength(stp::control_constants::AVOID_BALL_DISTANCE * 2);
    }
    Vector2 targetPosition = currentPosition + direction;
    if (FieldComputations::pointIsValidPosition(field, targetPosition, avoidObjects, stp::control_constants::OUT_OF_FIELD_MARGIN)) {
        return targetPosition;
    }
    int rotationStepDegrees = 10;
    int maxRotationDegrees = 90;
    for (int i = rotationStepDegrees; i <= maxRotationDegrees; i += rotationStepDegrees) {
        for (int sign : {1, -1}) {
            double rotation = sign * i * M_PI / 180;
            Vector2 rotatedDirection = direction.rotate(rotation);
            Vector2 potentialTargetPosition = currentPosition + rotatedDirection;
            if (FieldComputations::pointIsValidPosition(field, potentialTargetPosition, avoidObjects, stp::control_constants::OUT_OF_FIELD_MARGIN)) {
                return potentialTargetPosition;
            }
        }
    }
    return targetPosition;
}

double PositionControl::calculateScore(const rtt::world::World *world, const rtt::Field &field, std::optional<BB::CollisionData> &firstCollision,
                                       Trajectory2D &trajectoryAroundCollision, stp::AvoidObjects avoidObjects, double startTime) {
    double totalTime = trajectoryAroundCollision.getTotalTime();
    double score = totalTime + startTime;
    if (!firstCollision.has_value()) {
        return score;
    }

    score += 5;
    score += std::max(0.0, 3 - firstCollision.value().collisionTime - startTime);

    if (avoidObjects.shouldAvoidDefenseArea) {
        auto defenseAreaCollision = worldObjects.getFirstDefenseAreaCollision(field, trajectoryAroundCollision);
        if (defenseAreaCollision.has_value()) {
            score += std::max(0.0, 1 - defenseAreaCollision.value().collisionTime - startTime) * 10;
            score += 5;
        }
    }

    auto currentStrategyName = rtt::ai::GameStateManager::getCurrentGameState().getCommandId();
    if (currentStrategyName == RefCommand::BALL_PLACEMENT_THEM) {
        auto ballPlacementPos = rtt::ai::GameStateManager::getRefereeDesignatedPosition();
        auto startPositionBall = world->getWorld()->getBall()->get()->position;
        if ((startPositionBall - ballPlacementPos).length() < stp::control_constants::BALL_PLACEMENT_ALMOST_DONE_DISTANCE) {
            return score;
        }
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
    std::sort(intermediatePoints.begin(), intermediatePoints.end(),
              [&targetPosition](const Vector2 &a, const Vector2 &b) { return (a - targetPosition).length() < (b - targetPosition).length(); });
    timeStep *= 3;

    double bestScore = std::numeric_limits<double>::max();
    std::optional<Trajectory2D> bestTrajectory = std::nullopt;

    for (const auto &intermediatePoint : intermediatePoints) {
        Trajectory2D trajectoryToIntermediatePoint(currentPosition, currentVelocity, intermediatePoint, maxRobotVelocity, ai::Constants::MAX_ACC_UPPER());

        auto intermediatePathCollision = worldObjects.getFirstCollision(world, field, trajectoryToIntermediatePoint, computedPaths, robotId, avoidObjects, completedTimeSteps);
        double maxLoopTime = intermediatePathCollision.has_value() ? intermediatePathCollision.value().collisionTime : trajectoryToIntermediatePoint.getTotalTime();
        int numSteps = static_cast<int>(maxLoopTime / timeStep);
        for (int i = 0; i <= numSteps; ++i) {
            double loopTime = i * timeStep;
            Vector2 newStartPosition = trajectoryToIntermediatePoint.getPosition(loopTime);
            Vector2 newStartVelocity = trajectoryToIntermediatePoint.getVelocity(loopTime);
            Trajectory2D trajectoryAroundCollision(newStartPosition, newStartVelocity, targetPosition, maxRobotVelocity, ai::Constants::MAX_ACC_UPPER());
            auto firstCollision = worldObjects.getFirstCollision(world, field, trajectoryAroundCollision, computedPaths, robotId, avoidObjects, completedTimeSteps, loopTime);
            if (!firstCollision.has_value()) {
                trajectoryToIntermediatePoint.addTrajectory(trajectoryAroundCollision, loopTime);
                return trajectoryToIntermediatePoint;
            } else {
                double score = calculateScore(world, field, firstCollision, trajectoryAroundCollision, avoidObjects, loopTime);
                if (score < bestScore) {
                    bestScore = score;
                    bestTrajectory = trajectoryToIntermediatePoint;
                }
            }
        }
    }
    return bestTrajectory;
}

std::vector<Vector2> PositionControl::createIntermediatePoints(const rtt::Field &field, std::optional<BB::CollisionData> &firstCollision, Vector2 &targetPosition) {
    float angleBetweenIntermediatePoints = M_PI_4 / 2;
    float fieldHeight = field.playArea.height();
    float pointExtension = fieldHeight / 18;
    Vector2 collisionToTargetNormalized = (targetPosition - firstCollision->collisionPosition).normalize();
    Vector2 pointToDrawFrom = firstCollision->collisionPosition + collisionToTargetNormalized * pointExtension;

    std::vector<Vector2> intermediatePoints;
    float intermediatePointRadius = fieldHeight / 4;
    Vector2 pointToRotate = pointToDrawFrom + collisionToTargetNormalized * intermediatePointRadius;
    for (int i = -6; i < 7; i++) {
        if (i != 0) {
            // The slight offset to the angle makes sure the points are not symmetrically placed, this means we don't keep on switching between two points that are equally good
            // when there is a collision at time 0ss
            Vector2 intermediatePoint = pointToRotate.rotateAroundPoint(i * angleBetweenIntermediatePoints - 0.01, pointToDrawFrom);

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
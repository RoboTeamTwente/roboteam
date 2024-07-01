#include "control/positionControl/PositionControl.h"

#include "control/positionControl/BBTrajectories/BBTrajectory2D.h"
#include "control/positionControl/CollisionCalculations.h"
#include "gui/Out.h"
#include "roboteam_utils/Print.h"
#include "roboteam_utils/Random.h"
#include "world/World.hpp"

namespace rtt::ai::control {
std::vector<Vector2> PositionControl::getComputedPath(int ID) { return computedPaths[ID]; }

Vector2 PositionControl::computeAndTrackTrajectory(const world::World *world, const Field &field, int robotId, Vector2 currentPosition, Vector2 currentVelocity,
                                                   Vector2 targetPosition, double maxRobotVelocity, stp::AvoidObjects avoidObjects) {
    double timeStep = 0.1;
    auto [theirDefenseAreaMargin, ourDefenseAreaMargin] = FieldComputations::getDefenseAreaMargin();
    if (avoidObjects.shouldAvoidBall && (currentPosition - world->getWorld()->getBall()->get()->position).length() < ai::constants::AVOID_BALL_DISTANCE) {
        targetPosition = handleBallCollision(world, field, currentPosition, avoidObjects);
        computedTrajectories[robotId] = Trajectory2D(currentPosition, currentVelocity, targetPosition, maxRobotVelocity, ai::constants::MAX_ACC);
    } else if ((GameStateManager::getCurrentGameState().getCommandId() == RefCommand::BALL_PLACEMENT_THEM ||
                GameStateManager::getCurrentGameState().getCommandId() == RefCommand::PREPARE_FORCED_START) &&
               LineSegment(world->getWorld()->getBall()->get()->position, GameStateManager::getRefereeDesignatedPosition()).distanceToLine(currentPosition) <
                   ai::constants::AVOID_BALL_DISTANCE) {
        targetPosition = handleBallPlacementCollision(world, field, currentPosition, targetPosition, avoidObjects);
        computedTrajectories[robotId] = Trajectory2D(currentPosition, currentVelocity, targetPosition, maxRobotVelocity, ai::constants::MAX_ACC);
    } else if ((avoidObjects.shouldAvoidOurDefenseArea && FieldComputations::getDefenseArea(field, true, ourDefenseAreaMargin, 1).contains(currentPosition)) ||
               (avoidObjects.shouldAvoidTheirDefenseArea && theirDefenseAreaMargin > constants::ROBOT_RADIUS + constants::GO_TO_POS_ERROR_MARGIN &&
                FieldComputations::getDefenseArea(field, false, theirDefenseAreaMargin, 1).contains(currentPosition))) {
        targetPosition = handleDefenseAreaCollision(field, currentPosition);
        computedTrajectories[robotId] = Trajectory2D(currentPosition, currentVelocity, targetPosition, maxRobotVelocity, ai::constants::MAX_ACC);
    } else {
        computedTrajectories[robotId] = Trajectory2D(currentPosition, currentVelocity, targetPosition, maxRobotVelocity, ai::constants::MAX_ACC);
        auto hasCollsion = CollisionCalculations::isColliding(computedTrajectories[robotId], avoidObjects, field, robotId, world, computedPaths);
        if (hasCollsion) {
            computedTrajectories[robotId] = findNewTrajectory(world, field, robotId, currentPosition, currentVelocity, targetPosition, maxRobotVelocity, timeStep, avoidObjects);
        }
    }
    computedPaths[robotId] = computedTrajectories[robotId].getPathApproach(0.05);

    gui::Out::draw(
        {
            .label = "path_lines" + std::to_string(robotId),
            .color = proto::Drawing::MAGENTA,
            .method = proto::Drawing::LINES_CONNECTED,
            .category = proto::Drawing::PATH_PLANNING,
            .forRobotId = robotId,
            .thickness = 1,
        },
        computedPaths[robotId]);
    // Since our acceleration is 3.5 m/s^2, this means we will move if we need to move more than
    // 2*(1/2*3.5*0.145^2) (2 times the distance we move in 0.145/2s under constant (de)acceleration (1/2 * a * t^2))
    // This is 0.07m, so we will move if we need to move more than 0.07m
    return computedTrajectories[robotId].getVelocity(0.145);
}

Vector2 PositionControl::handleBallCollision(const world::World *world, const Field &field, Vector2 currentPosition, stp::AvoidObjects avoidObjects) {
    auto ballPos = world->getWorld()->getBall()->get()->position;
    auto direction = currentPosition - ballPos;
    Vector2 targetPosition = currentPosition + direction.stretchToLength(constants::AVOID_BALL_DISTANCE * 2);
    if (FieldComputations::pointIsValidPosition(field, targetPosition, avoidObjects, constants::OUT_OF_FIELD_MARGIN)) {
        return targetPosition;
    }
    int rotationStepDegrees = 10;
    int maxRotationDegrees = 330;
    for (int i = rotationStepDegrees; i <= maxRotationDegrees; i += rotationStepDegrees) {
        for (int sign : {1, -1}) {
            double rotation = sign * i * M_PI / 180;
            Vector2 rotatedDirection = direction.stretchToLength(constants::AVOID_BALL_DISTANCE * 2).rotate(rotation);
            Vector2 potentialTargetPosition = currentPosition + rotatedDirection;
            if (FieldComputations::pointIsValidPosition(field, potentialTargetPosition, avoidObjects, constants::OUT_OF_FIELD_MARGIN)) {
                return potentialTargetPosition;
            }
        }
    }
    return currentPosition + direction.stretchToLength(constants::AVOID_BALL_DISTANCE * 2).rotate(M_PI / 2);
}

Vector2 PositionControl::handleBallPlacementCollision(const world::World *world, const Field &field, Vector2 currentPosition, Vector2 targetPosition,
                                                      stp::AvoidObjects avoidObjects) {
    if ((targetPosition - currentPosition).length() < constants::AVOID_BALL_DISTANCE * 2) {
        return targetPosition;
    }
    auto placementPos = GameStateManager::getRefereeDesignatedPosition();
    auto ballPos = world->getWorld()->getBall()->get()->position;
    auto direction = (placementPos - ballPos).stretchToLength(constants::AVOID_BALL_DISTANCE * 2);
    direction = direction.rotate((currentPosition - ballPos).cross(placementPos - ballPos) < 0 ? M_PI / 2 : -M_PI / 2);
    auto distance = LineSegment(ballPos, placementPos).distanceToLine(currentPosition);
    Vector2 targetPositionOne = currentPosition + direction.stretchToLength(constants::AVOID_BALL_DISTANCE - distance);
    Vector2 targetPositionTwo = currentPosition - direction.stretchToLength(constants::AVOID_BALL_DISTANCE + distance);
    bool targetOneIsValid = FieldComputations::pointIsValidPosition(field, targetPositionOne, avoidObjects, constants::OUT_OF_FIELD_MARGIN);
    bool targetTwoIsValid = FieldComputations::pointIsValidPosition(field, targetPositionTwo, avoidObjects, constants::OUT_OF_FIELD_MARGIN);
    if (targetOneIsValid && !targetTwoIsValid) {
        return currentPosition + direction;
    }
    if (!targetOneIsValid && targetTwoIsValid) {
        return currentPosition - direction;
    }
    if (targetOneIsValid && targetTwoIsValid) {
        auto intersectionTargetOne = LineSegment(targetPositionOne, targetPosition).doesIntersect(LineSegment(ballPos, placementPos));
        auto intersectionTargetTwo = LineSegment(targetPositionTwo, targetPosition).doesIntersect(LineSegment(ballPos, placementPos));
        if (!intersectionTargetOne && intersectionTargetTwo) {
            return currentPosition + direction;
        } else if (intersectionTargetOne && !intersectionTargetTwo) {
            return currentPosition - direction;
        }
        return ((targetPosition - targetPositionOne).length() + constants::AVOID_BALL_DISTANCE - distance) <
                       ((targetPosition - targetPositionTwo).length() + constants::AVOID_BALL_DISTANCE + distance)
                   ? currentPosition + direction
                   : currentPosition - direction;
    } else {
        return currentPosition + direction;
    }
}

Vector2 PositionControl::handleDefenseAreaCollision(const Field &field, Vector2 currentPosition) {
    Vector2 ourGoalCenter = field.leftGoalArea.rightLine().center();
    Vector2 theirGoalCenter = field.rightGoalArea.leftLine().center();
    Vector2 closestGoalCenter = (currentPosition - ourGoalCenter).length() < (currentPosition - theirGoalCenter).length() ? ourGoalCenter : theirGoalCenter;
    Vector2 targetPosition = currentPosition + (currentPosition - closestGoalCenter).stretchToLength(constants::AVOID_BALL_DISTANCE * 2);
    return targetPosition;
}

Trajectory2D PositionControl::findNewTrajectory(const world::World *world, const Field &field, int robotId, Vector2 &currentPosition, Vector2 &currentVelocity,
                                                Vector2 &targetPosition, double maxRobotVelocity, double timeStep, stp::AvoidObjects avoidObjects) {
    std::vector<Vector2> normalizedPoints = generateNormalizedPoints(robotId);
    timeStep *= 2;
    Vector2 startToDest = targetPosition - currentPosition;
    for (const auto &normalizedPoint : normalizedPoints) {
        auto intermediatePoint = startToDest.rotate(normalizedPoint.angle()) * normalizedPoint.length() + currentPosition;
        Trajectory2D trajectoryToIntermediatePoint(currentPosition, currentVelocity, intermediatePoint, maxRobotVelocity, ai::constants::MAX_ACC);
        auto timeTillFirstCollision = CollisionCalculations::getFirstCollisionTime(trajectoryToIntermediatePoint, avoidObjects, field, robotId, world, computedPaths);
        double maxLoopTime = timeTillFirstCollision != -1 ? timeTillFirstCollision - 0.1 : trajectoryToIntermediatePoint.getTotalTime();
        int numSteps = static_cast<int>(maxLoopTime / timeStep);
        for (int i = 0; i <= numSteps; ++i) {
            double loopTime = i * timeStep;
            Vector2 newStartPosition = trajectoryToIntermediatePoint.getPosition(loopTime);
            Vector2 newStartVelocity = trajectoryToIntermediatePoint.getVelocity(loopTime);
            Trajectory2D trajectoryFromIntermediateToTarget(newStartPosition, newStartVelocity, targetPosition, maxRobotVelocity, ai::constants::MAX_ACC);
            auto hasCollision = CollisionCalculations::isColliding(trajectoryFromIntermediateToTarget, avoidObjects, field, robotId, world, computedPaths);
            if (!hasCollision) {
                trajectoryToIntermediatePoint.addTrajectory(trajectoryFromIntermediateToTarget, loopTime);
                lastUsedNormalizedPoints[robotId] = normalizedPoint;
                return trajectoryToIntermediatePoint;
            }
        }
    }
    lastUsedNormalizedPoints.erase(robotId);
    return Trajectory2D(currentPosition, currentVelocity, currentPosition, maxRobotVelocity, ai::constants::MAX_ACC);
    ;
}

std::vector<Vector2> PositionControl::generateNormalizedPoints(int robotId) {
    std::vector<Vector2> normalizedPoints;
    for (int i = 0; i < NUM_SUB_DESTINATIONS; i++) {
        double angleRange = MAX_ANGLE - MIN_ANGLE;
        double angle = SimpleRandom::getDouble(-angleRange, angleRange);
        angle += std::copysign(1.0, angle) * MIN_ANGLE;
        double scale = SimpleRandom::getDouble(MIN_SCALE, MAX_SCALE);
        normalizedPoints.push_back(Vector2(scale * cos(angle), scale * sin(angle)));
    }
    if (lastUsedNormalizedPoints.contains(robotId)) {
        normalizedPoints.push_back(lastUsedNormalizedPoints[robotId]);
    }
    std::sort(normalizedPoints.begin(), normalizedPoints.end(), [](const Vector2 &a, const Vector2 &b) { return std::abs(a.angle()) < std::abs(b.angle()); });
    return normalizedPoints;
}
}  // namespace rtt::ai::control
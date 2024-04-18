#include "control/positionControl/PositionControl.h"

#include "control/positionControl/BBTrajectories/BBTrajectory2D.h"
#include "gui/Out.h"
#include "roboteam_utils/Print.h"
#include "roboteam_utils/Random.h"
#include "world/World.hpp"

namespace rtt::ai::control {
std::vector<Vector2> PositionControl::getComputedPath(int ID) { return computedPaths[ID]; }

bool PositionControl::shouldRecalculatePath(const Vector2 &currentPosition, const Vector2 &targetPos, const Vector2 &currentVelocity, int robotId) {
    return computedPaths[robotId].empty() || PositionControlUtils::isTargetChanged(targetPos, computedPaths[robotId].back()) ||
           (currentVelocity != Vector2(0, 0) && collisionDetector.isCollisionBetweenPoints(currentPosition, computedPaths[robotId].front()));
}

void PositionControl::setRobotPositions(std::vector<Vector2> &robotPositions) { collisionDetector.setRobotPositions(robotPositions); }

rtt::ai::control::CommandCollision PositionControl::computeAndTrackTrajectory(const rtt::world::World *world, const rtt::Field &field, int robotId, Vector2 currentPosition,
                                                                              Vector2 currentVelocity, Vector2 targetPosition, double maxRobotVelocity, stp::PIDType pidType,
                                                                              stp::AvoidObjects avoidObjects) {
    std::optional<rtt::ai::control::CollisionData> firstCollision;
    rtt::ai::control::CommandCollision commandCollision = {};

    if (shouldRecalculateTrajectory(world, field, robotId, targetPosition, currentPosition, avoidObjects)) {
        double timeStep = 0.1;
        completedTimeSteps[robotId] = 0;
        if (avoidObjects.shouldAvoidBall && (currentPosition - world->getWorld()->getBall()->get()->position).length() < avoidObjects.avoidBallDist) {
            targetPosition = handleBallCollision(world, field, currentPosition, avoidObjects);
            computedTrajectories[robotId] = Trajectory2D(currentPosition, currentVelocity, targetPosition, maxRobotVelocity, ai::Constants::MAX_ACC_UPPER());
        } else if ((rtt::ai::GameStateManager::getCurrentGameState().getCommandId() == RefCommand::BALL_PLACEMENT_THEM ||
                    rtt::ai::GameStateManager::getCurrentGameState().getCommandId() == RefCommand::PREPARE_FORCED_START) &&
                   LineSegment(world->getWorld()->getBall()->get()->position, rtt::ai::GameStateManager::getRefereeDesignatedPosition()).distanceToLine(currentPosition) <
                       ai::stp::control_constants::AVOID_BALL_DISTANCE) {
            targetPosition = handleBallPlacementCollision(world, field, currentPosition, avoidObjects);
            computedTrajectories[robotId] = Trajectory2D(currentPosition, currentVelocity, targetPosition, maxRobotVelocity, ai::Constants::MAX_ACC_UPPER());
        } else if (avoidObjects.shouldAvoidDefenseArea &&
                   (rtt::ai::FieldComputations::getDefenseArea(field, true, std::get<1>(rtt::ai::FieldComputations::getDefenseAreaMargin()), 1).contains(currentPosition) ||
                    rtt::ai::FieldComputations::getDefenseArea(field, false, std::get<1>(rtt::ai::FieldComputations::getDefenseAreaMargin()), 1).contains(currentPosition))) {
            targetPosition = handleDefenseAreaCollision(field, currentPosition);
            computedTrajectories[robotId] = Trajectory2D(currentPosition, currentVelocity, targetPosition, maxRobotVelocity, ai::Constants::MAX_ACC_UPPER());
        } else {
            computedTrajectories[robotId] = Trajectory2D(currentPosition, currentVelocity, targetPosition, maxRobotVelocity, ai::Constants::MAX_ACC_UPPER());
            firstCollision = worldObjects.getFirstCollision(world, field, computedTrajectories[robotId], computedPaths, robotId, avoidObjects, completedTimeSteps);
            if (firstCollision.has_value()) {
                computedTrajectories[robotId] =
                    findNewTrajectory(world, field, robotId, currentPosition, currentVelocity, firstCollision, targetPosition, maxRobotVelocity, timeStep, avoidObjects);
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

    Position trackingVelocity = pathTrackingAlgorithmBBT.trackPathForwardAngle(currentPosition, currentVelocity, computedPathsPosVel[robotId], robotId, pidType);
    commandCollision.robotCommand.velocity = {trackingVelocity.x, trackingVelocity.y};
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
    direction = direction.rotate((currentPosition - ballPos).cross(placementPos - ballPos) < 0 ? M_PI / 2 : -M_PI / 2);
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

rtt::Vector2 PositionControl::handleDefenseAreaCollision(const rtt::Field &field, Vector2 currentPosition) {
    Vector2 ourGoalCenter = field.leftGoalArea.rightLine().center();
    Vector2 theirGoalCenter = field.rightGoalArea.leftLine().center();
    Vector2 closestGoalCenter = (currentPosition - ourGoalCenter).length() < (currentPosition - theirGoalCenter).length() ? ourGoalCenter : theirGoalCenter;
    Vector2 targetPosition = currentPosition + (currentPosition - closestGoalCenter).stretchToLength(stp::control_constants::AVOID_BALL_DISTANCE * 2);
    return targetPosition;
}

Trajectory2D PositionControl::findNewTrajectory(const rtt::world::World *world, const rtt::Field &field, int robotId, Vector2 &currentPosition, Vector2 &currentVelocity,
                                                std::optional<rtt::ai::control::CollisionData> &firstCollision, Vector2 &targetPosition, double maxRobotVelocity, double timeStep,
                                                stp::AvoidObjects avoidObjects) {
    std::vector<Vector2> normalizedPoints = generateNormalizedPoints(robotId);
    timeStep *= 2;
    Vector2 startToDest = targetPosition - currentPosition;
    for (const auto &normalizedPoint : normalizedPoints) {
        auto intermediatePoint = startToDest.rotate(normalizedPoint.angle()) * normalizedPoint.length() + currentPosition;
        Trajectory2D trajectoryToIntermediatePoint(currentPosition, currentVelocity, intermediatePoint, maxRobotVelocity, ai::Constants::MAX_ACC_UPPER());
        auto intermediatePathCollision = worldObjects.getFirstCollision(world, field, trajectoryToIntermediatePoint, computedPaths, robotId, avoidObjects, completedTimeSteps);
        double maxLoopTime = intermediatePathCollision.has_value() ? intermediatePathCollision.value().collisionTime : trajectoryToIntermediatePoint.getTotalTime();
        int numSteps = static_cast<int>(maxLoopTime / timeStep);
        for (int i = 0; i <= numSteps; ++i) {
            double loopTime = i * timeStep;
            Vector2 newStartPosition = trajectoryToIntermediatePoint.getPosition(loopTime);
            Vector2 newStartVelocity = trajectoryToIntermediatePoint.getVelocity(loopTime);
            Trajectory2D trajectoryFromIntermediateToTarget(newStartPosition, newStartVelocity, targetPosition, maxRobotVelocity, ai::Constants::MAX_ACC_UPPER());
            auto firstCollision =
                worldObjects.getFirstCollision(world, field, trajectoryFromIntermediateToTarget, computedPaths, robotId, avoidObjects, completedTimeSteps, loopTime);
            if (!firstCollision.has_value()) {
                trajectoryToIntermediatePoint.addTrajectory(trajectoryFromIntermediateToTarget, loopTime);
                lastUsedNormalizedPoints[robotId] = normalizedPoint;
                return trajectoryToIntermediatePoint;
            }
        }
    }
    lastUsedNormalizedPoints.erase(robotId);
    return Trajectory2D(currentPosition, currentVelocity, currentPosition, maxRobotVelocity, ai::Constants::MAX_ACC_UPPER());
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

bool PositionControl::shouldRecalculateTrajectory(const rtt::world::World *world, const rtt::Field &field, int robotId, Vector2 targetPosition, const Vector2 &currentPosition,
                                                  ai::stp::AvoidObjects avoidObjects) {
    if (!computedTrajectories.contains(robotId) ||
        (computedPaths.contains(robotId) && !computedPaths[robotId].empty() &&
         (targetPosition - computedPaths[robotId][computedPaths[robotId].size() - 1]).length() > stp::control_constants::GO_TO_POS_ERROR_MARGIN) ||
        worldObjects.getFirstCollision(world, field, computedTrajectories[robotId], computedPaths, robotId, avoidObjects, completedTimeSteps).has_value() ||
        ((computedPaths[robotId][0] - currentPosition).length() > 0.5)) {
        return true;
    }

    return false;
}

}  // namespace rtt::ai::control
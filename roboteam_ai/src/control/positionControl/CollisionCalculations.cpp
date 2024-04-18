#include "control/positionControl/CollisionCalculations.h"

#include <algorithm>

#include "stp/constants/ControlConstants.h"
#include "world/World.hpp"
#include "world/WorldData.hpp"

namespace rtt::ai::control {

bool CollisionCalculations::isTrajectoryAcceptable(const Trajectory2D &Trajectory) {
    // TODO: Implement the logic to check if the trajectory is acceptable.
    // This is a placeholder implementation that always returns true.
    // A trajectory should be acceptable if:
    // No collision with motionless objects
    // No collision with moving objects
    return true;
}

double CollisionCalculations::getFirstCollisionTimeMotionlessObject(const Trajectory2D &Trajectory, stp::AvoidObjects avoidObjects, const rtt::Field &field, int &robotId,
                                                                    int &completedTimeSteps) {
    auto pathPoints = Trajectory.getPathApproach(0.1);
    auto maxCheckPoints = std::min(pathPoints.size(), static_cast<size_t>(7) + completedTimeSteps);

    auto [theirDefenseAreaMargin, ourDefenseAreaMargin] = rtt::ai::FieldComputations::getDefenseAreaMargin();
    const auto &ourDefenseArea = rtt::ai::FieldComputations::getDefenseArea(field, true, ourDefenseAreaMargin, 1);
    const auto &theirDefenseArea = rtt::ai::FieldComputations::getDefenseArea(field, false, theirDefenseAreaMargin, 1);

    auto leftGoalTopPost = LineSegment(field.leftGoalArea.topLeft(), field.leftGoalArea.topRight());
    auto leftGoalBottomPost = LineSegment(field.leftGoalArea.bottomLeft(), field.leftGoalArea.bottomRight());
    auto leftGoalBackPost = LineSegment(field.rightGoalArea.topLeft(), field.rightGoalArea.bottomLeft());
    auto rightGoalTopPost = LineSegment(field.rightGoalArea.topLeft(), field.rightGoalArea.topRight());
    auto rightGoalBottomPost = LineSegment(field.rightGoalArea.bottomLeft(), field.rightGoalArea.bottomRight());
    auto rightGoalBackPost = LineSegment(field.rightGoalArea.topRight(), field.rightGoalArea.bottomRight());

    for (int checkPoint = 1 + completedTimeSteps; checkPoint <= maxCheckPoints; checkPoint += 1) {
        auto pathLine = LineSegment(pathPoints[checkPoint - 1], pathPoints[checkPoint]);
        if (avoidObjects.shouldAvoidGoalPosts) {
            if (pathLine.closestDistanceToLineSegment(leftGoalTopPost) < Constants::ROBOT_RADIUS() ||
                pathLine.closestDistanceToLineSegment(leftGoalBottomPost) < Constants::ROBOT_RADIUS() ||
                pathLine.closestDistanceToLineSegment(leftGoalBackPost) < Constants::ROBOT_RADIUS() ||
                pathLine.closestDistanceToLineSegment(rightGoalTopPost) < Constants::ROBOT_RADIUS() ||
                pathLine.closestDistanceToLineSegment(rightGoalBottomPost) < Constants::ROBOT_RADIUS() ||
                pathLine.closestDistanceToLineSegment(rightGoalBackPost) < Constants::ROBOT_RADIUS()) {
                return checkPoint * 0.1;
            }
        }
        if (avoidObjects.shouldAvoidOurDefenseArea) {
            if (ourDefenseArea.contains(pathPoints[checkPoint]) || ourDefenseArea.contains(pathPoints[checkPoint - 1]) || ourDefenseArea.doesIntersect(pathLine)) {
                return checkPoint * 0.1;
            }
        }
        if (avoidObjects.shouldAvoidTheirDefenseArea) {
            if (theirDefenseArea.contains(pathPoints[checkPoint]) || theirDefenseArea.contains(pathPoints[checkPoint - 1]) || theirDefenseArea.doesIntersect(pathLine)) {
                return checkPoint * 0.1;
            }
        }
        if (avoidObjects.shouldAvoidOutOfField) {
            if (!field.playArea.contains(pathPoints[checkPoint], stp::control_constants::OUT_OF_FIELD_MARGIN)) {
                return checkPoint * 0.1;
            }
        }
    }
    return -1.0;  // Return -1 if no collision occurred
}

bool CollisionCalculations::isCollidingWithMotionlessObject(const Trajectory2D &Trajectory, stp::AvoidObjects avoidObjects, const rtt::Field &field, int &robotId,
                                                            int &completedTimeSteps) {
    double collisionTime = getFirstCollisionTimeMotionlessObject(Trajectory, avoidObjects, field, robotId, completedTimeSteps);
    return collisionTime != -1.0;
}

double CollisionCalculations::getFirstCollisionTimeMovingObject(const Trajectory2D &Trajectory, stp::AvoidObjects avoidObjects, const rtt::Field &field, int &robotId,
                                                                int &completedTimeSteps) {
    auto pathPoints = Trajectory.getPathApproach(0.1);
    auto maxCheckPoints = std::min(pathPoints.size(), static_cast<size_t>(7) + completedTimeSteps);

    for (int checkPoint = 1 + completedTimeSteps; checkPoint <= maxCheckPoints; checkPoint += 1) {
        auto pathLine = LineSegment(pathPoints[checkPoint - 1], pathPoints[checkPoint]);
        if (avoidObjects.shouldAvoidOurRobots) {
            return 0;
        }
        if (avoidObjects.shouldAvoidTheirRobots) {
            return 0;
        }
        if (avoidObjects.shouldAvoidBall) {
            return 0;
        }
        if (GameStateManager::getCurrentGameState().getCommandId() == RefCommand::BALL_PLACEMENT_THEM ||
            GameStateManager::getCurrentGameState().getCommandId() == RefCommand::PREPARE_FORCED_START) {
            return 0;
        }
    }
    return -1.0;  // Return -1 if no collision occurred
}

bool CollisionCalculations::isCollidingWithMovingObject(const Trajectory2D &Trajectory, stp::AvoidObjects avoidObjects, const rtt::Field &field, int &robotId,
                                                        int &completedTimeSteps) {
    double collisionTime = getFirstCollisionTimeMovingObject(Trajectory, avoidObjects, field, robotId, completedTimeSteps);
    return collisionTime != -1.0;
}

}  // namespace rtt::ai::control
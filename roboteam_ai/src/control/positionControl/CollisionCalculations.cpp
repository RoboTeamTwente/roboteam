#include "control/positionControl/CollisionCalculations.h"

#include <algorithm>

#include "stp/constants/ControlConstants.h"
#include "world/World.hpp"
#include "world/WorldData.hpp"

namespace rtt::ai::control {

double CollisionCalculations::getFirstCollisionTimeMotionlessObject(const Trajectory2D &Trajectory, stp::AvoidObjects avoidObjects, const Field &field) {
    auto pathPoints = Trajectory.getPathApproach(0.1);
    auto maxCheckPoints = std::min(pathPoints.size(), static_cast<size_t>(7));

    auto [theirDefenseAreaMargin, ourDefenseAreaMargin] = FieldComputations::getDefenseAreaMargin();
    const auto &ourDefenseArea = FieldComputations::getDefenseArea(field, true, ourDefenseAreaMargin, 1);
    const auto &theirDefenseArea = FieldComputations::getDefenseArea(field, false, theirDefenseAreaMargin, 1);

    auto leftGoalTopPost = LineSegment(field.leftGoalArea.topLeft(), field.leftGoalArea.topRight());
    auto leftGoalBottomPost = LineSegment(field.leftGoalArea.bottomLeft(), field.leftGoalArea.bottomRight());
    auto leftGoalBackPost = LineSegment(field.rightGoalArea.topLeft(), field.rightGoalArea.bottomLeft());
    auto rightGoalTopPost = LineSegment(field.rightGoalArea.topLeft(), field.rightGoalArea.topRight());
    auto rightGoalBottomPost = LineSegment(field.rightGoalArea.bottomLeft(), field.rightGoalArea.bottomRight());
    auto rightGoalBackPost = LineSegment(field.rightGoalArea.topRight(), field.rightGoalArea.bottomRight());

    for (int checkPoint = 1; checkPoint < static_cast<int>(maxCheckPoints); checkPoint += 1) {
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

bool CollisionCalculations::isCollidingWithMotionlessObject(const Trajectory2D &Trajectory, stp::AvoidObjects avoidObjects, const Field &field) {
    double collisionTime = getFirstCollisionTimeMotionlessObject(Trajectory, avoidObjects, field);
    return collisionTime != -1.0;
}

double CollisionCalculations::getFirstCollisionTimeMovingObject(const Trajectory2D &Trajectory, stp::AvoidObjects avoidObjects, int &robotId, const world::World *world,
                                                                const std::unordered_map<int, std::vector<Vector2>> &computedPaths) {
    auto pathPoints = Trajectory.getPathApproach(0.1);
    auto maxCheckPoints = std::min(pathPoints.size(), static_cast<size_t>(7));

    const std::vector<world::view::RobotView> &theirRobots = world->getWorld()->getThem();
    const std::vector<world::view::RobotView> &ourRobots = world->getWorld()->getUs();

    double maxVel = GameStateManager::getCurrentGameState().getRuleSet().getMaxRobotVel();

    for (int checkPoint = 1; checkPoint < static_cast<int>(maxCheckPoints); checkPoint += 1) {
        auto pathLine = LineSegment(pathPoints[checkPoint - 1], pathPoints[checkPoint]);
        double velocityOurRobot = Trajectory.getVelocity(checkPoint * 0.1).length();
        auto positionOurRobot = Trajectory.getPosition(checkPoint * 0.1);
        double additionalMargin = std::pow(std::min(maxVel, velocityOurRobot) / maxVel, 2) * 0.2;
        if (velocityOurRobot > 0.7 && avoidObjects.shouldAvoidOurRobots) {
            for (const auto &ourOtherRobot : ourRobots) {
                const int &ourOtherRobotId = ourOtherRobot->getId();
                if (ourOtherRobotId == robotId) {
                    continue;
                }
                const auto computedPathsIt = computedPaths.find(ourOtherRobotId);
                if (computedPathsIt == computedPaths.end()) {
                    const Vector2 &ourOtherRobotPos = ourOtherRobot->getPos();
                    if ((ourOtherRobotPos - positionOurRobot).length() < 2 * Constants::ROBOT_RADIUS() + additionalMargin) {
                        return checkPoint * 0.1;
                    }
                } else {
                    Vector2 ourOtherRobotPos;
                    if (checkPoint < static_cast<int>(computedPathsIt->second.size())) {
                        // This should be their checkPoint instead of ours
                        ourOtherRobotPos = computedPathsIt->second[checkPoint];
                    } else {
                        ourOtherRobotPos = computedPathsIt->second.back();
                    }
                    if ((ourOtherRobotPos - positionOurRobot).length() < 2 * Constants::ROBOT_RADIUS()) {
                        return checkPoint * 0.1;
                    }
                }
            }
        }
        if (velocityOurRobot > 0.7 && avoidObjects.shouldAvoidTheirRobots) {
            if (std::any_of(theirRobots.begin(), theirRobots.end(), [&](const auto &theirRobot) {
                    return (theirRobot->getPos() + theirRobot->getVel() * 0.1 * checkPoint - positionOurRobot).length() < 2 * Constants::ROBOT_RADIUS() + additionalMargin;
                })) {
                return checkPoint * 0.1;
            }
        }
        if (avoidObjects.shouldAvoidBall) {
            auto ballPosition = FieldComputations::getBallPositionAtTime(*world->getWorld()->getBall()->get(), checkPoint * 0.1);
            if ((ballPosition - positionOurRobot).length() < stp::control_constants::AVOID_BALL_DISTANCE + additionalMargin) {
                return checkPoint * 0.1;
            }
        }
        if (GameStateManager::getCurrentGameState().getCommandId() == RefCommand::BALL_PLACEMENT_THEM ||
            GameStateManager::getCurrentGameState().getCommandId() == RefCommand::PREPARE_FORCED_START) {
            auto ballPosition = FieldComputations::getBallPositionAtTime(*world->getWorld()->getBall()->get(), checkPoint * 0.1);
            auto ballPlacementPosition = GameStateManager::getRefereeDesignatedPosition();
            LineSegment ballPlacementLine(ballPosition, ballPlacementPosition);
            if (ballPlacementLine.distanceToLine(positionOurRobot) < stp::control_constants::AVOID_BALL_DISTANCE + additionalMargin) {
                return checkPoint * 0.1;
            }
        }
    }
    return -1.0;  // Return -1 if no collision occurred
}

bool CollisionCalculations::isCollidingWithMovingObject(const Trajectory2D &Trajectory, stp::AvoidObjects avoidObjects, int &robotId, const world::World *world,
                                                        const std::unordered_map<int, std::vector<Vector2>> &computedPaths) {
    double collisionTime = getFirstCollisionTimeMovingObject(Trajectory, avoidObjects, robotId, world, computedPaths);
    return collisionTime != -1.0;
}

double CollisionCalculations::getFirstCollisionTime(const Trajectory2D &Trajectory, stp::AvoidObjects avoidObjects, const Field &field, int &robotId, const world::World *world,
                                                    const std::unordered_map<int, std::vector<Vector2>> &computedPaths) {
    double collisionTime = getFirstCollisionTimeMotionlessObject(Trajectory, avoidObjects, field);
    if (collisionTime != -1.0) {
        return collisionTime;
    }
    return getFirstCollisionTimeMovingObject(Trajectory, avoidObjects, robotId, world, computedPaths);
}

bool CollisionCalculations::isColliding(const Trajectory2D &Trajectory, stp::AvoidObjects avoidObjects, const Field &field, int &robotId, const world::World *world,
                                        const std::unordered_map<int, std::vector<Vector2>> &computedPaths) {
    return isCollidingWithMotionlessObject(Trajectory, avoidObjects, field) || isCollidingWithMovingObject(Trajectory, avoidObjects, robotId, world, computedPaths);
}

}  // namespace rtt::ai::control
#include "control/positionControl/CollisionCalculations.h"

#include <algorithm>

#include "utilities/Constants.h"
#include "world/World.hpp"
#include "world/WorldData.hpp"

namespace rtt::ai::control {

double CollisionCalculations::getFirstCollisionTimeMotionlessObject(const Trajectory2D &Trajectory, stp::AvoidObjects avoidObjects, const Field &field) {
    auto pathPoints = Trajectory.getPathApproach(0.05);
    auto maxCheckPoints = std::min(pathPoints.size(), static_cast<size_t>(14));

    auto [theirDefenseAreaMargin, ourDefenseAreaMargin] = FieldComputations::getDefenseAreaMargin();
    const auto &ourDefenseArea = FieldComputations::getDefenseArea(field, true, ourDefenseAreaMargin, 1);
    const auto &theirDefenseArea = FieldComputations::getDefenseArea(field, false, theirDefenseAreaMargin, 1);

    auto leftGoalTopPost = field.leftGoalArea.topLine();
    auto leftGoalBottomPost = field.leftGoalArea.bottomLine();
    auto leftGoalBackPost = field.leftGoalArea.leftLine();
    auto rightGoalTopPost = field.rightGoalArea.topLine();
    auto rightGoalBottomPost = field.rightGoalArea.bottomLine();
    auto rightGoalBackPost = field.rightGoalArea.rightLine();

    for (int checkPoint = 1; checkPoint < static_cast<int>(maxCheckPoints); checkPoint += 1) {
        auto pathLine = LineSegment(pathPoints[checkPoint - 1], pathPoints[checkPoint]);
        if (avoidObjects.shouldAvoidGoalPosts) {
            if (pathLine.closestDistanceToLineSegment(leftGoalTopPost) < constants::ROBOT_RADIUS ||
                pathLine.closestDistanceToLineSegment(leftGoalBottomPost) < constants::ROBOT_RADIUS ||
                pathLine.closestDistanceToLineSegment(leftGoalBackPost) < constants::ROBOT_RADIUS ||
                pathLine.closestDistanceToLineSegment(rightGoalTopPost) < constants::ROBOT_RADIUS ||
                pathLine.closestDistanceToLineSegment(rightGoalBottomPost) < constants::ROBOT_RADIUS ||
                pathLine.closestDistanceToLineSegment(rightGoalBackPost) < constants::ROBOT_RADIUS) {
            }
        }
        if (avoidObjects.shouldAvoidOurDefenseArea) {
            if (ourDefenseArea.contains(pathPoints[checkPoint]) || ourDefenseArea.contains(pathPoints[checkPoint - 1]) || ourDefenseArea.doesIntersect(pathLine)) {
                return checkPoint * 0.05;
            }
        }
        if (avoidObjects.shouldAvoidTheirDefenseArea && theirDefenseAreaMargin > constants::ROBOT_RADIUS + constants::GO_TO_POS_ERROR_MARGIN) {
            if (theirDefenseArea.contains(pathPoints[checkPoint]) || theirDefenseArea.contains(pathPoints[checkPoint - 1]) || theirDefenseArea.doesIntersect(pathLine)) {
                return checkPoint * 0.05;
            }
        }
        if (avoidObjects.shouldAvoidOutOfField) {
            if (!field.playArea.contains(pathPoints[checkPoint], constants::OUT_OF_FIELD_MARGIN)) {
                return checkPoint * 0.05;
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
    auto pathPoints = Trajectory.getPathApproach(0.05);
    auto maxCheckPoints = std::min(pathPoints.size(), static_cast<size_t>(7));

    const std::vector<world::view::RobotView> &theirRobots = world->getWorld()->getThem();
    const std::vector<world::view::RobotView> &ourRobots = world->getWorld()->getUs();

    double maxVel = GameStateManager::getCurrentGameState().getRuleSet().getMaxRobotVel();

    for (int checkPoint = 1; checkPoint < static_cast<int>(maxCheckPoints); checkPoint += 1) {
        auto pathLine = LineSegment(pathPoints[checkPoint - 1], pathPoints[checkPoint]);
        double velocityOurRobot = Trajectory.getVelocity(checkPoint * 0.05).length();
        auto positionOurRobot = Trajectory.getPosition(checkPoint * 0.05);
        double additionalMargin = std::pow(std::min(3.5, velocityOurRobot) / 3.5, 2) * 0.2;
        if (velocityOurRobot > 0.7 && avoidObjects.shouldAvoidOurRobots) {
            for (const auto &ourOtherRobot : ourRobots) {
                const int &ourOtherRobotId = ourOtherRobot->getId();
                if (ourOtherRobotId == robotId) {
                    continue;
                }

                const auto computedPathsIt = computedPaths.find(ourOtherRobotId);
                // If the path of the other robot is not computed, we assume it is not moving
                if (computedPathsIt == computedPaths.end()) {
                    const Vector2 &ourOtherRobotPos = ourOtherRobot->getPos();
                    if ((ourOtherRobotPos - positionOurRobot).length() < 2 * constants::ROBOT_RADIUS + additionalMargin) {
                        return checkPoint * 0.05;
                    }
                } else {
                    LineSegment pathLineOtherRobot;
                    if (checkPoint < static_cast<int>(computedPathsIt->second.size())) {
                        pathLineOtherRobot = LineSegment(computedPathsIt->second[checkPoint - 1], computedPathsIt->second[checkPoint]);
                    } else {
                        pathLineOtherRobot = LineSegment(computedPathsIt->second.back(), computedPathsIt->second.back());
                    }
                    if (pathLineOtherRobot.closestDistanceToLineSegment(pathLine) < 2 * constants::ROBOT_RADIUS + additionalMargin) {
                        return checkPoint * 0.05;
                    }
                }
            }
        } else if (avoidObjects.shouldAvoidOurRobots) {
            for (const auto &ourOtherRobot : ourRobots) {
                const int &ourOtherRobotId = ourOtherRobot->getId();
                if (ourOtherRobotId == robotId) {
                    continue;
                }
                const Vector2 &ourOtherRobotPos = ourOtherRobot->getPos();
                if ((ourOtherRobotPos - positionOurRobot).length() < 2 * constants::ROBOT_RADIUS) {
                    return checkPoint * 0.05;
                }
            }
        }
        if (velocityOurRobot > 0.7 && avoidObjects.shouldAvoidTheirRobots) {
            if (std::any_of(theirRobots.begin(), theirRobots.end(), [&](const auto &theirRobot) {
                    return LineSegment(theirRobot->getPos() + theirRobot->getVel() * 0.05 * (checkPoint - 1), theirRobot->getPos() + theirRobot->getVel() * 0.05 * checkPoint)
                               .closestDistanceToLineSegment(pathLine) < 2 * constants::ROBOT_RADIUS + additionalMargin;
                })) {
                return checkPoint * 0.05;
            }
        } else if (avoidObjects.shouldAvoidTheirRobots) {
            if (std::any_of(theirRobots.begin(), theirRobots.end(),
                            [&](const auto &theirRobot) { return (theirRobot->getPos() - positionOurRobot).length() < 1.8 * constants::CENTER_TO_FRONT; })) {
                return checkPoint * 0.05;
            }
        }
        if (avoidObjects.shouldAvoidBall) {
            auto ballPosition = FieldComputations::getBallPositionAtTime(*world->getWorld()->getBall()->get(), checkPoint * 0.05);
            if ((ballPosition - positionOurRobot).length() < constants::AVOID_BALL_DISTANCE + additionalMargin) {
                return checkPoint * 0.05;
            }
        }
        if (GameStateManager::getCurrentGameState().getCommandId() == RefCommand::BALL_PLACEMENT_THEM ||
            GameStateManager::getCurrentGameState().getCommandId() == RefCommand::PREPARE_FORCED_START) {
            auto ballPlacementPosition = GameStateManager::getRefereeDesignatedPosition();
            bool isBallPlacementCollision = true;
            for (int i = checkPoint; i < checkPoint + 20; i++) {
                auto ballPosition = FieldComputations::getBallPositionAtTime(*world->getWorld()->getBall()->get(), checkPoint * 0.05);
                if (i >= static_cast<int>(pathPoints.size())) {
                    isBallPlacementCollision = false;
                    break;
                }
                auto positionOurRobot = Trajectory.getPosition(i * 0.05);
                auto ballPlacementLine = LineSegment(ballPlacementPosition, ballPosition);
                if (ballPlacementLine.distanceToLine(positionOurRobot) > constants::AVOID_BALL_DISTANCE + additionalMargin) {
                    isBallPlacementCollision = false;
                    break;
                }
            }
            if (isBallPlacementCollision) {
                return checkPoint * 0.05;
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
    double collisionTimeMotionless = getFirstCollisionTimeMotionlessObject(Trajectory, avoidObjects, field);
    double collisionTimeMoving = getFirstCollisionTimeMovingObject(Trajectory, avoidObjects, robotId, world, computedPaths);

    return (collisionTimeMotionless == -1.0 && collisionTimeMoving == -1.0) ? -1.0
           : (collisionTimeMotionless == -1.0)                              ? collisionTimeMoving
           : (collisionTimeMoving == -1.0)                                  ? collisionTimeMotionless
                                                                            : std::min(collisionTimeMotionless, collisionTimeMoving);
}

bool CollisionCalculations::isColliding(const Trajectory2D &Trajectory, stp::AvoidObjects avoidObjects, const Field &field, int &robotId, const world::World *world,
                                        const std::unordered_map<int, std::vector<Vector2>> &computedPaths) {
    return isCollidingWithMotionlessObject(Trajectory, avoidObjects, field) || isCollidingWithMovingObject(Trajectory, avoidObjects, robotId, world, computedPaths);
}

}  // namespace rtt::ai::control
#include "stp/computations/InterceptionComputations.h"

#include <roboteam_utils/Grid.h>
#include <roboteam_utils/Tube.h>

#include <roboteam_utils/Field.hpp>

#include "roboteam_utils/Hungarian.h"
#include "stp/Play.hpp"
#include "stp/computations/ComputationManager.h"
#include "stp/computations/PositionScoring.h"
#include "world/World.hpp"

namespace rtt::ai::stp {

KeeperInterceptionInfo InterceptionComputations::calculateKeeperInterceptionInfo(const world::World *world, const world::view::RobotView &keeper) noexcept {
    KeeperInterceptionInfo keeperInterceptionInfo;
    auto field = world->getField().value();
    auto maxRobotVelocity = GameStateManager::getCurrentGameState().getRuleSet().getMaxRobotVel();
    int LineOfSightScore = 80;
    Vector2 futureBallPosition;
    Vector2 ballPosition = world->getWorld()->getBall()->get()->position;
    for (double loopTime = 0.1; loopTime < 1; loopTime += 0.1) {
        futureBallPosition = FieldComputations::getBallPositionAtTime(*(world->getWorld()->getBall()->get()), loopTime);
        // If the ball is out of the field or the LoS score is too low, we don't intercept
        if (PositionScoring::scorePosition(futureBallPosition, gen::LineOfSight, field, world).score < LineOfSightScore ||
            !field.playArea.contains(futureBallPosition, constants::BALL_RADIUS)) {
            return keeperInterceptionInfo;
        }
        // Only intercept if the ball will be in the defense area
        if (field.leftDefenseArea.contains(futureBallPosition)) {
            // If we are already close to the linesegment, project the position to prevent always moving to the 0.1 second mark
            if (LineSegment(ballPosition, futureBallPosition).distanceToLine(keeper->getPos()) < 1.5 * constants::ROBOT_RADIUS) {
                keeperInterceptionInfo.interceptLocation = LineSegment(ballPosition, futureBallPosition).project(keeper->getPos());
                keeperInterceptionInfo.canIntercept = true;
                return keeperInterceptionInfo;
            }
            // If we can reach the ball in time, we will intercept
            if (Trajectory2D(keeper->getPos(), keeper->getVel(), futureBallPosition, maxRobotVelocity, ai::constants::MAX_ACC, ai::constants::MAX_JERK_DEFAULT, keeper->getId())
                    .getTotalTime() < loopTime) {
                keeperInterceptionInfo.interceptLocation = futureBallPosition;
                keeperInterceptionInfo.canIntercept = true;
                return keeperInterceptionInfo;
            }
        }
    }
    return keeperInterceptionInfo;
}

InterceptionInfo InterceptionComputations::calculateInterceptionInfo(const std::vector<world::view::RobotView> &ourRobots, const world::World *world) {
    InterceptionInfo interceptionInfo;
    auto maxRobotVelocity = GameStateManager::getCurrentGameState().getRuleSet().getMaxRobotVel();
    auto ballPosition = world->getWorld()->getBall()->get()->position;
    auto pastBallPosition = ballPosition;
    auto futureBallPosition = ballPosition;
    auto ballVelocity = world->getWorld()->getBall()->get()->velocity;
    bool shouldReturn = false;
    interceptionInfo.interceptLocation = ballPosition;
    const LineSegment ballTrajectory(world->getWorld()->getBall()->get()->position, world->getWorld()->getBall()->get()->expectedEndPosition);
    auto interceptRobot = calculateTheirBallInterception(world, ballTrajectory);

    // Helper function to calculate the intercept info for a given target position
    auto calculateIntercept = [&](const Vector2 &targetPosition, const Vector2 &targetVelocity) {
        // Check if they are able to intercept the ball at the part of the trajectory we are checking
        if (interceptRobot && (*interceptRobot - pastBallPosition).length() < (futureBallPosition - pastBallPosition).length()) {
            auto minDistance = (*interceptRobot - pastBallPosition).length();
            auto robotCloseToBallPos = *interceptRobot;
            if ((world->getWorld()->getRobotClosestToBall(world::us)->get()->getPos() - pastBallPosition).length() < minDistance) {
                interceptionInfo.interceptLocation =
                    LineSegment(futureBallPosition, pastBallPosition).project(world->getWorld()->getRobotClosestToBall(world::us)->get()->getPos());
            } else {
                interceptionInfo.interceptLocation = robotCloseToBallPos;
            }
            double minTimeToTarget = std::numeric_limits<double>::max();
            for (const auto &robot : ourRobots) {
                auto trajectory = Trajectory2D(robot->getPos(), robot->getVel(), interceptionInfo.interceptLocation, maxRobotVelocity, ai::constants::MAX_ACC,
                                               ai::constants::MAX_JERK_DEFAULT, robot->getId());
                if (trajectory.getTotalTime() < minTimeToTarget) {
                    minTimeToTarget = trajectory.getTotalTime();
                    interceptionInfo.interceptId = robot->getId();
                    interceptionInfo.timeToIntercept = minTimeToTarget;
                }
            }
            interceptionInfo.interceptVelocity = Vector2(0, 0);
            shouldReturn = true;
            return;
        }

        for (const auto &robot : ourRobots) {
            auto ballToRobot = (robot->getPos() - ballPosition);
            auto ballToFutPos = (futureBallPosition - ballPosition);
            // FutPos and Robot are on opposite sides of the ball, don't consider this robot
            if (ballToRobot.dot(ballToFutPos) < 0) {
                continue;
            }
            // If the robot is already close to the line, project it's position onto the line to prevent always moving to the 0.1 second mark
            if (LineSegment(pastBallPosition, futureBallPosition).distanceToLine(robot->getPos()) < 1.5 * constants::ROBOT_RADIUS) {
                interceptionInfo.interceptLocation = LineSegment(pastBallPosition, futureBallPosition).project(robot->getPos());
                interceptionInfo.interceptVelocity = Vector2(0, 0);
                interceptionInfo.interceptId = robot->getId();
                interceptionInfo.timeToIntercept = 0;
                interceptionInfo.isInterceptable = true;
                return;
            }
        }

        double minTimeToTarget = std::numeric_limits<double>::max();
        for (const auto &robot : ourRobots) {
            Vector2 usedTargetVelocity;
            auto robotToBall = (ballPosition - robot->getPos());
            auto angle = Angle(robotToBall).shortestAngleDiff(Angle(targetVelocity));
            // TODO ROBOCUP 2024: TWEAK THIS NOT MAGIC NUMBER
            if (std::abs(angle) < M_PI / 4) {
                usedTargetVelocity = targetVelocity;
            } else {
                usedTargetVelocity = Vector2(0, 0);
            }
            auto trajectory = Trajectory2D(robot->getPos(), robot->getVel(), targetPosition, usedTargetVelocity, maxRobotVelocity, ai::constants::MAX_ACC,
                                           ai::constants::MAX_JERK_OVERSHOOT, robot->getId());
            if (trajectory.getTotalTime() < minTimeToTarget) {
                minTimeToTarget = trajectory.getTotalTime();
                interceptionInfo.interceptLocation = targetPosition;
                interceptionInfo.interceptVelocity = usedTargetVelocity;
                interceptionInfo.interceptId = robot->getId();
                interceptionInfo.timeToIntercept = minTimeToTarget;
                auto theirClosestToBall = world->getWorld()->getRobotClosestToBall(world::them);
                if (!theirClosestToBall || (robot->getPos() - targetPosition).length() < (theirClosestToBall->get()->getPos() - targetPosition).length()) {
                    interceptionInfo.isInterceptable = true;
                }
            }
        }
    };

    // If the ball is not moving, we use the current ball position
    if (ballVelocity.length() <= constants::BALL_STILL_VEL) {
        calculateIntercept(ballPosition, Vector2(0, 0));
        interceptionInfo.interceptVelocity = Vector2(0, 0);
        return interceptionInfo;
    }

    for (double loopTime = 0.1; loopTime < 1; loopTime += 0.1) {
        futureBallPosition = FieldComputations::getBallPositionAtTime(*(world->getWorld()->getBall()->get()), loopTime);
        pastBallPosition = FieldComputations::getBallPositionAtTime(*(world->getWorld()->getBall()->get()), loopTime - 0.1);

        // If futureBallPos and pastBallPos are in the defense area, continue since interception is not possible
        if ((world->getField().value().leftDefenseArea.contains(futureBallPosition) && world->getField().value().leftDefenseArea.contains(pastBallPosition)) ||
            (world->getField().value().rightDefenseArea.contains(futureBallPosition) && world->getField().value().rightDefenseArea.contains(pastBallPosition))) {
            pastBallPosition = futureBallPosition;
            continue;
        } else if (world->getField().value().leftDefenseArea.contains(futureBallPosition) || world->getField().value().rightDefenseArea.contains(futureBallPosition)) {
            // project futureBallPos to where it enters the defense area
            auto intersections = FieldComputations::getDefenseArea(world->getField().value(), world->getField().value().leftDefenseArea.contains(futureBallPosition), 0, 0.5)
                                     .intersections({pastBallPosition, futureBallPosition});
            if (intersections.size() == 1) {
                futureBallPosition = intersections.at(0);
            }
        } else if (world->getField().value().leftDefenseArea.contains(pastBallPosition) || world->getField().value().rightDefenseArea.contains(pastBallPosition)) {
            // project pastBallPos to where it exits the defense area
            auto intersections = FieldComputations::getDefenseArea(world->getField().value(), world->getField().value().leftDefenseArea.contains(pastBallPosition), 0, 0.5)
                                     .intersections({pastBallPosition, futureBallPosition});
            if (intersections.size() == 1) {
                pastBallPosition = intersections.at(0);
            }
        }

        // If the ball is out of the field, we intercept at the projected position in the field, unless the ball is already out of the field
        if (!world->getField().value().playArea.contains(futureBallPosition, constants::BALL_RADIUS)) {
            if (world->getField().value().playArea.contains(ballPosition, constants::BALL_RADIUS)) {
                futureBallPosition =
                    FieldComputations::projectPointIntoFieldOnLine(world->getField().value(), futureBallPosition, ballPosition, futureBallPosition, constants::BALL_RADIUS);
                calculateIntercept(futureBallPosition, Vector2(0, 0));
            } else {
                calculateIntercept(ballPosition, Vector2(0, 0));
            }
            return interceptionInfo;
        }
        // TODO ROBOCUP 2024: PLEASE FIX :{}
        auto ballVelFuture = (futureBallPosition - pastBallPosition) / 0.1;
        calculateIntercept(futureBallPosition, ballVelFuture);
        // If any robot can intercept the ball in time, return that info
        if (loopTime >= interceptionInfo.timeToIntercept) {
            interceptionInfo.isInterceptable = true;
            return interceptionInfo;
        } else if (shouldReturn) {
            return interceptionInfo;
        }
    }
    return interceptionInfo;
}

int InterceptionComputations::getKeeperId(const std::vector<world::view::RobotView> &possibleRobots, const world::World *world) {
    auto keeperId = GameStateManager::getCurrentGameState().keeperId;
    auto keeperIt = std::find_if(possibleRobots.begin(), possibleRobots.end(), [keeperId](const auto &bot) { return bot->getId() == keeperId; });
    if (keeperIt != possibleRobots.end()) {
        return (*keeperIt)->getId();
    }
    auto keeper = world->getWorld()->getRobotClosestToPoint(world->getField().value().leftGoalArea.rightLine().center(), possibleRobots);
    if (keeper) {
        return keeper->get()->getId();
    }
    return -1;  // Should never reach this point unless there are no robots in the field
}

InterceptionInfo InterceptionComputations::calculateInterceptionInfoForKickingRobots(const std::vector<world::view::RobotView> &robots, const world::World *world) {
    InterceptionInfo interceptionInfo;
    auto possibleRobots = robots;

    // Remove robots that cannot kick
    std::erase_if(possibleRobots, [](const world::view::RobotView &rbv) { return !constants::ROBOT_HAS_KICKER(rbv->getId()); });

    // If there is no robot that can kick, return empty
    if (possibleRobots.empty()) return interceptionInfo;

    // If there is at least one, pick the one that can reach the ball the fastest
    interceptionInfo = InterceptionComputations::calculateInterceptionInfo(possibleRobots, world);

    // Remove robots that cannot detect the ball themselves (so no ballSensor or dribblerEncoder)
    auto numRobots = possibleRobots.size();
    std::erase_if(possibleRobots, [](const world::view::RobotView &rbv) { return !constants::ROBOT_HAS_WORKING_DRIBBLER_ENCODER(rbv->getId()); });

    // If no robot can detect the ball, the previous closest robot that can only kick is the best one
    if (possibleRobots.empty()) return interceptionInfo;

    // But if there is one, the current best passer will be the one that can reach the ball the fastest
    if (numRobots != possibleRobots.size()) interceptionInfo = InterceptionComputations::calculateInterceptionInfo(possibleRobots, world);

    return interceptionInfo;
}

InterceptionInfo InterceptionComputations::calculateInterceptionInfoExcludingKeeperAndCarded(const world::World *world) noexcept {
    auto ourRobots = world->getWorld()->getUs();
    // Remove the keeper and the robot that has a card from the list
    auto keeperId = GameStateManager::getCurrentGameState().keeperId;
    auto cardId = GameStateManager::getCurrentGameState().cardId;
    ourRobots.erase(std::remove_if(ourRobots.begin(), ourRobots.end(), [keeperId, cardId](const auto &robot) { return robot->getId() == keeperId || robot->getId() == cardId; }),
                    ourRobots.end());

    return calculateInterceptionInfoForKickingRobots(ourRobots, world);
}

std::optional<Vector2> InterceptionComputations::calculateTheirBallInterception(const world::World *world, rtt::LineSegment ballTrajectory) noexcept {
    auto ballOpt = world->getWorld()->getBall();
    if (!ballOpt) return std::nullopt;
    auto ball = ballOpt.value();
    std::optional<Vector2> closestInterceptionPoint = std::nullopt;
    double minimumDistanceToBall = std::numeric_limits<double>::max();
    for (const auto &opponentRobot : world->getWorld()->get()->getThem()) {
        auto projectedPoint = ballTrajectory.project(opponentRobot->getPos());
        if (projectedPoint.dist(opponentRobot->getPos()) < 0.5) {
            double distanceToBallFromProjectedPoint = projectedPoint.dist(ball->position);
            if (distanceToBallFromProjectedPoint < minimumDistanceToBall) {
                minimumDistanceToBall = distanceToBallFromProjectedPoint;
                Vector2 adjustmentVector = (ball->position - projectedPoint).normalize() * constants::CENTER_TO_FRONT;
                closestInterceptionPoint = projectedPoint + adjustmentVector;
            }
        }
    }
    return closestInterceptionPoint;
}

}  // namespace rtt::ai::stp
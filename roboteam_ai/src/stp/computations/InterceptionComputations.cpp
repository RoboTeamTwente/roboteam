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
            !field.playArea.contains(futureBallPosition, control_constants::BALL_RADIUS)) {
            return keeperInterceptionInfo;
        }
        // Only intercept if the ball will be in the defense area
        if (field.leftDefenseArea.contains(futureBallPosition)) {
            // If we are already close to the linesegment, project the position to prevent always moving to the 0.1 second mark
            if (LineSegment(ballPosition, futureBallPosition).distanceToLine(keeper->getPos()) < 1.5 * control_constants::ROBOT_RADIUS) {
                keeperInterceptionInfo.interceptLocation = LineSegment(ballPosition, futureBallPosition).project(keeper->getPos());
                keeperInterceptionInfo.canIntercept = true;
                return keeperInterceptionInfo;
            }
            // If we can reach the ball in time, we will intercept
            if (Trajectory2D(keeper->getPos(), keeper->getVel(), futureBallPosition, maxRobotVelocity, ai::Constants::MAX_ACC()).getTotalTime() < loopTime) {
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
    int LineOfSightScore = 50;
    auto maxRobotVelocity = GameStateManager::getCurrentGameState().getRuleSet().getMaxRobotVel();
    auto ballPosition = world->getWorld()->getBall()->get()->position;
    auto pastBallPosition = ballPosition;
    auto futureBallPosition = ballPosition;
    auto ballVelocity = world->getWorld()->getBall()->get()->velocity;
    interceptionInfo.interceptLocation = ballPosition;

    // Helper function to calculate the intercept info for a given target position
    auto calculateIntercept = [&](const Vector2 &targetPosition) {
        // If the LoS score is too low
        if (PositionScoring::scorePosition(futureBallPosition, gen::LineOfSight, world->getField().value(), world).score < LineOfSightScore) {
            auto interceptPosition = futureBallPosition;
            auto minDistance = std::numeric_limits<double>::max();
            auto theirRobots = world->getWorld()->getThem();
            Vector2 robotCloseToBallPos;
            for (const auto &theirRobot : theirRobots) {
                auto distance = LineSegment(futureBallPosition, pastBallPosition).distanceToLine(theirRobot->getPos());
                if (distance < minDistance) {
                    minDistance = distance;
                    robotCloseToBallPos = theirRobot->getPos();
                }
            }
            if ((world->getWorld()->getRobotClosestToBall(world::us)->get()->getPos() - pastBallPosition).length() < minDistance) {
                interceptionInfo.interceptLocation =
                    LineSegment(futureBallPosition, pastBallPosition).project(world->getWorld()->getRobotClosestToBall(world::us)->get()->getPos());
            } else {
                interceptionInfo.interceptLocation =
                    robotCloseToBallPos + (world->getWorld()->getBall()->get()->position - robotCloseToBallPos).stretchToLength(control_constants::ROBOT_RADIUS * 3);
            }
            return;
        }

        for (const auto &robot : ourRobots) {
            // If the robot is already close to the line, project it's position onto the line to prevent always moving to the 0.1 second mark
            if (LineSegment(pastBallPosition, futureBallPosition).distanceToLine(robot->getPos()) < 1.5 * control_constants::ROBOT_RADIUS) {
                interceptionInfo.interceptLocation = LineSegment(pastBallPosition, futureBallPosition).project(robot->getPos());
                interceptionInfo.interceptId = robot->getId();
                interceptionInfo.timeToIntercept = 0;
                interceptionInfo.isInterceptable = true;
                return;
            }
        }

        double minTimeToTarget = std::numeric_limits<double>::max();
        for (const auto &robot : ourRobots) {
            auto trajectory = Trajectory2D(robot->getPos(), robot->getVel(), targetPosition, maxRobotVelocity, ai::Constants::MAX_ACC());
            if (trajectory.getTotalTime() < minTimeToTarget) {
                minTimeToTarget = trajectory.getTotalTime();
                interceptionInfo.interceptId = robot->getId();
                interceptionInfo.interceptLocation = targetPosition;
                interceptionInfo.timeToIntercept = minTimeToTarget;
                auto theirClosestToBall = world->getWorld()->getRobotClosestToBall(world::them);
                if (!theirClosestToBall || (robot->getPos() - targetPosition).length() < (theirClosestToBall->get()->getPos() - targetPosition).length()) {
                    interceptionInfo.isInterceptable = true;
                }
            }
        }
    };

    // If the ball is not moving, we use the current ball position
    if (ballVelocity.length() <= control_constants::BALL_STILL_VEL) {
        calculateIntercept(ballPosition);
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
            auto intersections = FieldComputations::getDefenseArea(world->getField().value(), world->getField().value().leftDefenseArea.contains(futureBallPosition), 0, 0)
                                     .intersections({pastBallPosition, futureBallPosition});
            if (intersections.size() == 1) {
                futureBallPosition = intersections.at(0);
            }
        } else if (world->getField().value().leftDefenseArea.contains(pastBallPosition) || world->getField().value().rightDefenseArea.contains(pastBallPosition)) {
            // project pastBallPos to where it exits the defense area
            auto intersections = FieldComputations::getDefenseArea(world->getField().value(), world->getField().value().leftDefenseArea.contains(pastBallPosition), 0, 0)
                                     .intersections({pastBallPosition, futureBallPosition});
            if (intersections.size() == 1) {
                pastBallPosition = intersections.at(0);
            }
        }

        // If the ball is out of the field, we intercept at the projected position in the field, unless the ball is already out of the field
        if (!world->getField().value().playArea.contains(futureBallPosition, control_constants::BALL_RADIUS)) {
            if (world->getField().value().playArea.contains(ballPosition, control_constants::BALL_RADIUS)) {
                futureBallPosition = FieldComputations::projectPointInField(world->getField().value(), futureBallPosition);
                calculateIntercept(futureBallPosition);
            } else {
                calculateIntercept(ballPosition);
            }
            return interceptionInfo;
        }

        calculateIntercept(futureBallPosition);
        // If any robot can intercept the ball in time, return that info
        if (loopTime >= interceptionInfo.timeToIntercept) {
            interceptionInfo.isInterceptable = true;
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
    std::erase_if(possibleRobots, [](const world::view::RobotView &rbv) { return !Constants::ROBOT_HAS_KICKER(rbv->getId()); });

    // If there is no robot that can kick, return empty
    if (possibleRobots.empty()) return interceptionInfo;

    // If there is at least one, pick the one that can reach the ball the fastest
    interceptionInfo = InterceptionComputations::calculateInterceptionInfo(possibleRobots, world);

    // Remove robots that cannot detect the ball themselves (so no ballSensor or dribblerEncoder)
    auto numRobots = possibleRobots.size();
    std::erase_if(possibleRobots, [](const world::view::RobotView &rbv) { return !Constants::ROBOT_HAS_WORKING_DRIBBLER_ENCODER(rbv->getId()); });

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
}  // namespace rtt::ai::stp
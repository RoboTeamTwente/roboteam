#include "stp/computations/InterceptionComputations.h"

#include <roboteam_utils/Grid.h>
#include <roboteam_utils/Tube.h>

#include <roboteam_utils/Field.hpp>

#include "roboteam_utils/Hungarian.h"
#include "stp/Play.hpp"
#include "stp/computations/ComputationManager.h"
#include "stp/computations/PassComputations.h"
#include "stp/computations/PositionScoring.h"
#include "world/World.hpp"

namespace rtt::ai::stp {

KeeperInterceptInfo InterceptionComputations::calculateKeeperInterceptionInfo(const world::World *world, const world::view::RobotView &keeper) noexcept {
    KeeperInterceptInfo keeperInterceptInfo;
    auto field = world->getField().value();
    auto maxRobotVelocity = GameStateManager::getCurrentGameState().getRuleSet().getMaxRobotVel();
    int interceptScore = 80;
    Vector2 futureBallPosition;
    Vector2 ballPosition = world->getWorld()->getBall()->get()->position;
    for (double loopTime = 0.1; loopTime < 1; loopTime += 0.1) {
        futureBallPosition = FieldComputations::getBallPositionAtTime(*(world->getWorld()->getBall()->get()), loopTime);
        // If the ball is out of the field or the LoS score is too low, we don't intercept
        if (PositionScoring::scorePosition(futureBallPosition, gen::LineOfSight, field, world).score < interceptScore ||
            !field.playArea.contains(futureBallPosition, control_constants::BALL_RADIUS)) {
            return keeperInterceptInfo;
        }
        // Only intercept if the ball will be in the defense area
        if (field.leftDefenseArea.contains(futureBallPosition)) {
            // If we are already close to the linesegment, project the position to prevent always moving to the 0.1 second mark
            if (LineSegment(ballPosition, futureBallPosition).distanceToLine(keeper->getPos()) < 1.5 * control_constants::ROBOT_RADIUS) {
                keeperInterceptInfo.interceptLocation = LineSegment(ballPosition, futureBallPosition).project(keeper->getPos());
                keeperInterceptInfo.canIntercept = true;
                return keeperInterceptInfo;
            }
            // If we can reach the ball in time, we will intercept
            if (Trajectory2D(keeper->getPos(), keeper->getVel(), futureBallPosition, maxRobotVelocity, ai::Constants::MAX_ACC_UPPER()).getTotalTime() < loopTime) {
                keeperInterceptInfo.interceptLocation = futureBallPosition;
                keeperInterceptInfo.canIntercept = true;
                return keeperInterceptInfo;
            }
        }
    }
    return keeperInterceptInfo;
}

// Bad naming lol
InterceptInfo InterceptionComputations::calculateInterceptionInfo(const std::vector<world::view::RobotView> &ourRobots, const world::World *world) {
    InterceptInfo interceptInfo;
    int interceptScore = 50;
    auto maxRobotVelocity = GameStateManager::getCurrentGameState().getRuleSet().getMaxRobotVel();
    auto ballPosition = world->getWorld()->getBall()->get()->position;
    auto pastBallPosition = ballPosition;
    auto futureBallPosition = ballPosition;
    auto ballVelocity = world->getWorld()->getBall()->get()->velocity;
    interceptInfo.interceptLocation = ballPosition;

    // Helper function to calculate the intercept info for a given target position
    auto calculateIntercept = [&](const Vector2 &targetPosition) {
        for (const auto &robot : ourRobots) {
            // If the robot is already close to the line, project it's position onto the line to prevent always moving to the 0.1 second mark
            if (LineSegment(pastBallPosition, futureBallPosition).distanceToLine(robot->getPos()) < 1.5 * control_constants::ROBOT_RADIUS) {
                interceptInfo.interceptLocation = LineSegment(pastBallPosition, futureBallPosition).project(robot->getPos());
                interceptInfo.interceptId = robot->getId();
                interceptInfo.timeToIntercept = 0;
                return;
            }
        }

        double minTimeToTarget = std::numeric_limits<double>::max();
        for (const auto &robot : ourRobots) {
            auto trajectory = Trajectory2D(robot->getPos(), robot->getVel(), targetPosition, maxRobotVelocity, ai::Constants::MAX_ACC_UPPER());
            if (trajectory.getTotalTime() < minTimeToTarget) {
                minTimeToTarget = trajectory.getTotalTime();
                interceptInfo.interceptId = robot->getId();
                interceptInfo.interceptLocation = targetPosition;
                interceptInfo.timeToIntercept = minTimeToTarget;
            }
        }
    };

    // If the ball is not moving, we use the current ball position
    if (ballVelocity.length() <= control_constants::BALL_STILL_VEL) {
        calculateIntercept(ballPosition);
        return interceptInfo;
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

        // If the LoS score is too low
        if (PositionScoring::scorePosition(futureBallPosition, gen::LineOfSight, world->getField().value(), world).score < interceptScore) {
            auto interceptPosition = futureBallPosition;
            auto minDistance = std::numeric_limits<double>::max();
            auto theirRobots = world->getWorld()->getThem();
            for (const auto &theirRobot : theirRobots) {
                auto distance = LineSegment(futureBallPosition, pastBallPosition).distanceToLine(theirRobot->getPos());
                if (distance < minDistance) {
                    minDistance = distance;
                    interceptPosition = LineSegment(futureBallPosition, pastBallPosition).project(theirRobot->getPos());
                }
            }
            calculateIntercept(interceptPosition);
            return interceptInfo;
        }

        // If the ball is out of the field, we intercept at the projected position in the field, unless the ball is already out of the field
        if (!world->getField().value().playArea.contains(futureBallPosition, control_constants::BALL_RADIUS)) {
            if (world->getField().value().playArea.contains(ballPosition, control_constants::BALL_RADIUS)) {
                futureBallPosition = FieldComputations::projectPointInField(world->getField().value(), futureBallPosition);
                calculateIntercept(futureBallPosition);
            } else {
                calculateIntercept(ballPosition);
            }
            return interceptInfo;
        }

        calculateIntercept(futureBallPosition);
        // If any robot can intercept the ball in time, return that info
        if (loopTime >= interceptInfo.timeToIntercept) {
            return interceptInfo;
        }
    }
    return interceptInfo;
}

InterceptInfo InterceptionComputations::calculateGetBallId(const world::World *world) noexcept {
    InterceptInfo interceptionInfo;

    auto ourRobots = world->getWorld()->getUs();
    auto keeperId = GameStateManager::getCurrentGameState().keeperId;
    ourRobots.erase(std::remove_if(ourRobots.begin(), ourRobots.end(), [keeperId](const auto &robot) { return robot->getId() == keeperId; }), ourRobots.end());

    auto possibleGetBaller = ourRobots;
    // Remove robots that cannot kick
    std::erase_if(possibleGetBaller, [](const world::view::RobotView &rbv) { return !Constants::ROBOT_HAS_KICKER(rbv->getId()); });

    if (possibleGetBaller.empty()) return interceptionInfo;
    // If there is at least one, pick the one that can reach the ball the fastest
    interceptionInfo = InterceptionComputations::calculateInterceptionInfo(possibleGetBaller, world);

    // Remove robots that cannot detect the ball themselves (so no ballSensor or dribblerEncoder)
    std::erase_if(possibleGetBaller, [](const world::view::RobotView &rbv) { return !Constants::ROBOT_HAS_WORKING_DRIBBLER_ENCODER(rbv->getId()); });

    // If no robot can detect the ball, the previous closest robot that can only kick is the best one
    if (possibleGetBaller.empty()) return interceptionInfo;
    // But if there is one, the current best passer will be the one that can reach the ball the fastest
    interceptionInfo = InterceptionComputations::calculateInterceptionInfo(possibleGetBaller, world);

    return interceptionInfo;
}
}  // namespace rtt::ai::stp
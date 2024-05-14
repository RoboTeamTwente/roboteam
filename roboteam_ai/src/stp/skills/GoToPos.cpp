#include "stp/skills/GoToPos.h"

#include "stp/computations/PositionComputations.h"
#include "world/World.hpp"

namespace rtt::ai::stp::skill {

Status GoToPos::onUpdate(const StpInfo &info) noexcept {
    auto robot = info.getRobot().value();
    auto field = info.getField().value();
    auto avoidObj = info.getObjectsToAvoid();
    auto targetPos = info.getPositionToMoveTo().value();
    auto roleName = info.getRoleName();
    auto ballLocation = info.getBall()->get()->position;
    RefCommand currentGameState = GameStateManager::getCurrentGameState().getCommandId();

    if (currentGameState == RefCommand::BALL_PLACEMENT_THEM) {
        auto ballPlacementPos = GameStateManager::getRefereeDesignatedPosition();
        auto robotToTarget = LineSegment(robot->getPos(), targetPos);
        auto ballToReferee = LineSegment(ballLocation, ballPlacementPos);
        if (robotToTarget.doesIntersect(ballToReferee)) {
            double distance1 = (robot->getPos() - ballLocation).length() + (ballLocation - targetPos).length();
            double distance2 = (robot->getPos() - ballPlacementPos).length() + (ballPlacementPos - targetPos).length();
            targetPos = (distance1 < distance2) ? ballLocation - (ballPlacementPos - ballLocation).stretchToLength(0.8)
                                                : ballPlacementPos - (robot->getPos() - ballPlacementPos).stretchToLength(0.8);
            targetPos = targetPos - (robot->getPos() - targetPos).stretchToLength(0.8);
        }
    }

    if (!FieldComputations::pointIsValidPosition(field, targetPos, avoidObj) && roleName != "ball_placer") {
        targetPos = FieldComputations::projectPointToValidPosition(field, targetPos, avoidObj);
    }

    if (avoidObj.shouldAvoidOurRobots || avoidObj.shouldAvoidTheirRobots) {
        targetPos = PositionComputations::calculateAvoidRobotsPosition(targetPos, info.getCurrentWorld(), robot->getId(), avoidObj, field);
    }

    if (roleName != "ball_placer" && (avoidObj.shouldAvoidBall || currentGameState == RefCommand::BALL_PLACEMENT_US || currentGameState == RefCommand::BALL_PLACEMENT_THEM ||
                                      currentGameState == RefCommand::BALL_PLACEMENT_US_DIRECT)) {
        targetPos = PositionComputations::calculateAvoidBallPosition(targetPos, ballLocation, field);
    }

    command.velocity = info.getCurrentWorld()->getRobotPositionController()->computeAndTrackTrajectory(info.getCurrentWorld(), field, robot->getId(), robot->getPos(),
                                                                                                       robot->getVel(), targetPos, info.getMaxRobotVelocity(), avoidObj);

    auto distanceToTarget = (robot->getPos() - targetPos).length();
    command.targetAngle = distanceToTarget <= 0.5 ? info.getAngle() : robot->getAngle() + rtt::Angle(0.5 / distanceToTarget * (info.getAngle() - robot->getAngle()));

    // Clamp and set dribbler speed
    int targetDribblerPercentage = std::clamp(info.getDribblerSpeed(), 0, 100);
    command.dribblerSpeed = targetDribblerPercentage / 100.0 * stp::control_constants::MAX_DRIBBLER_CMD;

    // set command ID
    command.id = robot->getId();

    // forward the generated command to the ControlModule, for checking and limiting
    forwardRobotCommand();

    // Check if successful
    auto distanceError = (robot->getPos() - targetPos).length();
    if (distanceError <= stp::control_constants::GO_TO_POS_ERROR_MARGIN ||
        (robot->hasBall() && distanceError <= stp::control_constants::BALL_PLACEMENT_MARGIN - stp::control_constants::GO_TO_POS_ERROR_MARGIN)) {
        return Status::Success;
    } else {
        return Status::Running;
    }
}

const char *GoToPos::getName() { return "Go To Position"; }

}  // namespace rtt::ai::stp::skill
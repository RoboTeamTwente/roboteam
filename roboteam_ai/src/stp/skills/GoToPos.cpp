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
            Vector2 point1 = ballLocation - (ballPlacementPos - ballLocation).stretchToLength(0.8);
            Vector2 point2 = ballPlacementPos - (robot->getPos() - ballPlacementPos).stretchToLength(0.8);
            bool point1InField = field.playArea.contains(point1, constants::OUT_OF_FIELD_MARGIN);
            bool point2InField = field.playArea.contains(point2, constants::OUT_OF_FIELD_MARGIN);
            if (point1InField && point2InField) {
                targetPos = (distance1 < distance2) ? point1 : point2;
            } else if (point1InField) {
                targetPos = point1;
            } else if (point2InField) {
                targetPos = point2;
            } else {
                targetPos = FieldComputations::projectPointToValidPosition(field, targetPos, avoidObj);
            }
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
        targetPos = PositionComputations::calculateAvoidBallPosition(targetPos, ballLocation, field, avoidObj);
    }

    // TODO:: SET JERK TO A NORMAL VALUE, from stp info or something
    double mJerk = 12;
    double maxJerk = mJerk / (0.04 * 60);
    auto [vel, acc] = info.getCurrentWorld()->getRobotPositionController()->computeAndTrackTrajectory(info.getCurrentWorld(), field, robot->getId(), robot->getPos(),
                                                                                                      robot->getVel(), targetPos, info.getMaxRobotVelocity(), maxJerk, avoidObj);
    command.velocity = vel;
    command.acceleration = acc;

    auto distanceToTarget = (robot->getPos() - targetPos).length();
    command.yaw = distanceToTarget <= 0.5 ? info.getYaw() : robot->getYaw() + rtt::Angle(0.5 / distanceToTarget * (info.getYaw() - robot->getYaw()));

    command.dribblerOn = info.getDribblerOn();

    // set command ID
    command.id = robot->getId();

    // forward the generated command to the ControlModule, for checking and limiting
    forwardRobotCommand();

    // Check if successful
    auto distanceError = (robot->getPos() - targetPos).length();
    if (robot->hasBall() && distanceError <= constants::BALL_PLACEMENT_MARGIN - constants::GO_TO_POS_ERROR_MARGIN) {
        return Status::Success;
    } else {
        return Status::Running;
    }
}

const char *GoToPos::getName() { return "Go To Position"; }

}  // namespace rtt::ai::stp::skill
//
// Created by jordi on 09-03-20.
//

#include "stp/skills/GoToPos.h"

#include "control/positionControl/BBTrajectories/WorldObjects.h"
#include "stp/computations/PositionComputations.h"
#include "world/World.hpp"

namespace rtt::ai::stp::skill {

Status GoToPos::onUpdate(const StpInfo &info) noexcept {
    Vector2 targetPos = info.getPositionToMoveTo().value();
    auto avoidObj = info.getObjectsToAvoid();
    std::string roleName = info.getRoleName();
    Vector2 ballLocation = info.getBall()->get()->position;
    if (!FieldComputations::pointIsValidPosition(info.getField().value(), targetPos, avoidObj) && roleName != "ball_placer") {
        targetPos = FieldComputations::projectPointToValidPosition(info.getField().value(), targetPos, avoidObj);
    }
    if (avoidObj.shouldAvoidOurRobots || avoidObj.shouldAvoidTheirRobots || avoidObj.notAvoidTheirRobotId != -1) {
        targetPos = PositionComputations::calculateAvoidRobotsPosition(targetPos, info.getCurrentWorld(), info.getRobot().value()->getId(), avoidObj, info.getField().value());
    }
    if (avoidObj.shouldAvoidBall) {
        targetPos = PositionComputations::calculateAvoidBallPosition(targetPos, ballLocation, info.getField().value());
    }

    rtt::BB::CommandCollision commandCollision;

    commandCollision = info.getCurrentWorld()->getRobotPositionController()->computeAndTrackTrajectory(
        info.getCurrentWorld(), info.getField().value(), info.getRobot().value()->getId(), info.getRobot().value()->getPos(), info.getRobot().value()->getVel(), targetPos,
        info.getMaxRobotVelocity(), info.getPidType().value(), avoidObj);

    auto &velocity = commandCollision.robotCommand.velocity;
    command.velocity = velocity.stretchToLength(std::clamp(velocity.length(), 0.0, info.getMaxRobotVelocity()));

    // TODO: Test with control peeps to see what angle works best when driving.
    // Driving and turning do not work well together, so we only turn when we are close to the target position.
    // This also avoids driving into the defense area when robots are moving just allong the edge of the defense area.
    if ((info.getRobot().value()->getPos() - targetPos).length() <= 0.5) {
        command.targetAngle = info.getAngle();
    } else {
        command.targetAngle = info.getRobot().value()->getAngle();
    }

    // Clamp and set dribbler speed
    int targetDribblerPercentage = std::clamp(info.getDribblerSpeed(), 0, 100);
    double targetDribblerSpeed = targetDribblerPercentage / 100.0 * stp::control_constants::MAX_DRIBBLER_CMD;

    // Set dribbler speed command
    command.dribblerSpeed = targetDribblerSpeed;

    // set command ID
    command.id = info.getRobot().value()->getId();

    // forward the generated command to the ControlModule, for checking and limiting
    forwardRobotCommand();

    // Check if successful
    if ((info.getRobot().value()->getPos() - targetPos).length() <= stp::control_constants::GO_TO_POS_ERROR_MARGIN ||
        (info.getRobot().value()->hasBall() && (info.getRobot().value()->getPos() - targetPos).length() <= stp::control_constants::BALL_PLACEMENT_MARGIN)) {
        return Status::Success;
    } else {
        return Status::Running;
    }
}

const char *GoToPos::getName() { return "Go To Position"; }

}  // namespace rtt::ai::stp::skill
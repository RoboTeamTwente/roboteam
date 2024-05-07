#include "stp/skills/Rotate.h"

#include "control/ControlUtils.h"
#include "stp/constants/ControlConstants.h"

namespace rtt::ai::stp::skill {

Status Rotate::onUpdate(const StpInfo &info) noexcept {
    auto robot = info.getRobot().value();
    auto targetAngle = info.getAngle();

    // Set angle command
    command.targetAngle = targetAngle;

    // Set velocity if robot has the ball
    if (robot->hasBall()) {
        command.velocity = Vector2(0.3, 0).rotate(robot->getAngle());
    }

    // Set dribbler speed command
    command.dribblerSpeed = std::clamp(info.getDribblerSpeed(), 0, 100) / 100.0 * stp::control_constants::MAX_DRIBBLER_CMD;

    // Set command ID
    command.id = robot->getId();

    // Forward the generated command to the ControlModule, for checking and limiting
    forwardRobotCommand();

    // Check if the robot is within the error margin
    double errorMargin = stp::control_constants::GO_TO_POS_ANGLE_ERROR_MARGIN * M_PI;
    if (robot->getAngle().shortestAngleDiff(targetAngle) < errorMargin) {
        withinMarginCount++;
    } else {
        withinMarginCount = 0;
    }

    // Check whether the robot has been within the margin
    return (withinMarginCount > 5) ? Status::Success : Status::Running;
}

const char *Rotate::getName() { return "Rotate"; }

}  // namespace rtt::ai::stp::skill
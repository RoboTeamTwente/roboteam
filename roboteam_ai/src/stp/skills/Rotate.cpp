#include "stp/skills/Rotate.h"

#include "control/ControlUtils.h"
#include "stp/constants/ControlConstants.h"

namespace rtt::ai::stp::skill {

Status Rotate::onUpdate(const StpInfo &info) noexcept {
    auto robot = info.getRobot().value();
    auto yaw = info.getYaw();

    // Set yaw command
    command.yaw = yaw;

    command.dribblerOn = info.getDribblerOn();

    // Set command ID
    command.id = robot->getId();

    // Forward the generated command to the ControlModule, for checking and limiting
    forwardRobotCommand();

    // Check if the robot is within the error margin
    double errorMargin = stp::control_constants::GO_TO_POS_ANGLE_ERROR_MARGIN * M_PI;
    if (robot->getYaw().shortestAngleDiff(yaw) < errorMargin) {
        withinMarginCount++;
    } else {
        withinMarginCount = 0;
    }

    // Check whether the robot has been within the margin
    return (withinMarginCount > 3) ? Status::Success : Status::Running;
}

const char *Rotate::getName() { return "Rotate"; }

}  // namespace rtt::ai::stp::skill
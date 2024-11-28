#include "stp/skills/OrbitAngular.h"

#include "utilities/Constants.h"
#include "utilities/GameSettings.h"

namespace rtt::ai::stp::skill {

Status OrbitAngular::onUpdate(const StpInfo &info) noexcept {
    auto robot = info.getRobot().value();
    auto ball = info.getBall()->get();

    // Calculate target and current yaws
    Angle currentYaw = robot->getYaw();
    Angle yaw = (info.getPositionToShootAt().value() - ball->position).toAngle();

    // Determine direction and speed factor
    int direction = yaw.rotateDirection(currentYaw) ? -1 : 1;
    double speedFactor = std::clamp(currentYaw.shortestAngleDiff(yaw) * 1.4 * M_PI, 0.0, M_PI);

    // Calculate target angular velocity and normal vector
    double targetAngularVelocity = direction * speedFactor;
    Vector2 normalVector = currentYaw.toVector2().rotate(-direction * M_PI_2);

    // Calculate target velocity
    Vector2 targetVelocity = normalVector * speedFactor * (constants::BALL_RADIUS + constants::CENTER_TO_FRONT);

    // Construct the robot command
    command.id = robot->getId();
    // command.velocity = targetVelocity;

    // target yaw is angular velocity times the time step (1/60th of a second) in basestation
    // in simulator, we multiple by 1/2.5 because of how the simulator works and the pid controller is tuned
    if (rtt::GameSettings::getRobotHubMode() == rtt::net::RobotHubMode::BASESTATION) {
        command.yaw = currentYaw + Angle(targetAngularVelocity * 1 / 6);
    } else {
        command.yaw = currentYaw + Angle(targetAngularVelocity * 1 / 2.5);
    }
    command.dribblerOn = true;

    forwardRobotCommand();  // Send the robot command

    // Check if the robot is within the error margin
    double errorMargin = constants::GO_TO_POS_ANGLE_ERROR_MARGIN * M_PI;
    if (currentYaw.shortestAngleDiff(yaw) < errorMargin) {
        withinMarginCount++;
    } else {
        withinMarginCount = 0;
    }

    // Check whether the robot has been within the margin
    return (withinMarginCount > 0) ? Status::Success : Status::Running;
}

const char *OrbitAngular::getName() { return "OrbitAngular"; }

}  // namespace rtt::ai::stp::skill
#include "stp/skills/OrbitAngular.h"

#include "stp/constants/ControlConstants.h"

namespace rtt::ai::stp::skill {

Status OrbitAngular::onUpdate(const StpInfo &info) noexcept {
    auto robot = info.getRobot().value();
    auto ball = info.getBall()->get();

    // Calculate target and current angles
    Angle currentAngle = robot->getAngle();
    Angle targetAngle = (info.getPositionToShootAt().value() - ball->position).toAngle();

    // Determine direction and speed factor
    int direction = targetAngle.rotateDirection(currentAngle) ? -1 : 1;
    double speedFactor = std::clamp(currentAngle.shortestAngleDiff(targetAngle) * 2 * M_PI, 0.0, M_PI);

    // Calculate target angular velocity and normal vector
    double targetAngularVelocity = direction * speedFactor;
    Vector2 normalVector = currentAngle.toVector2().rotate(-direction * M_PI_2);

    // Calculate target velocity
    Vector2 targetVelocity = normalVector * speedFactor * (stp::control_constants::BALL_RADIUS + stp::control_constants::ROBOT_RADIUS);

    // Construct the robot command
    command.id = robot->getId();
    command.velocity = targetVelocity;
    // Commented for the control code before the Schubert open on 3 April 2024. Possible wise to use again somewhere in the distant future.
    // command.useAngularVelocity = true;
    // command.targetAngularVelocity = targetAngularVelocity;
    command.targetAngle = targetAngle;
    command.dribblerSpeed = stp::control_constants::MAX_DRIBBLER_CMD;

    forwardRobotCommand();  // Send the robot command

    // Return success if the angle difference is within the margin
    if (currentAngle.shortestAngleDiff(targetAngle) < stp::control_constants::GO_TO_POS_ANGLE_ERROR_MARGIN * M_PI) {
        return Status::Success;
    } else {
        return Status::Running;
    }
}

const char *OrbitAngular::getName() { return "OrbitAngular"; }

}  // namespace rtt::ai::stp::skill
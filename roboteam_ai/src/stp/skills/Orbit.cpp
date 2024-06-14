#include "stp/skills/Orbit.h"

#include "utilities/Constants.h"

namespace rtt::ai::stp::skill {

Status Orbit::onUpdate(const StpInfo &info) noexcept {
    auto robot = info.getRobot().value();
    auto ball = info.getBall()->get();

    Vector2 directionVector = ball->position - robot->getPos();
    double normalAngle = directionVector.rotate(M_PI).rotate(M_PI_2).angle();
    Angle yaw = (info.getPositionToShootAt().value() - ball->position).toAngle();

    double margin = 1.5 * constants::ROBOT_RADIUS + constants::BALL_RADIUS;
    double adjustDistance = robot->getDistanceToBall() - margin;

    // Get the direction of movement, counterclockwise or clockwise
    auto direction = Angle(directionVector).rotateDirection(yaw) ? -1.0 : 1.0;

    double error = yaw.shortestAngleDiff(directionVector.angle());
    error = error * direction;

    // Use PID controller to determine desired velocity multiplier
    auto multiplier = velPid.getOutput(error, 0);

    // Velocity consists of a part to create the circular movement, and an adjustment for distance to the ball
    Vector2 targetVelocity;
    targetVelocity.x = cos(normalAngle) * multiplier + 3 * cos(directionVector.toAngle()) * adjustDistance;
    targetVelocity.y = sin(normalAngle) * multiplier + 3 * sin(directionVector.toAngle()) * adjustDistance;

    auto maxVel = directionVector.length() * 4;
    maxVel = std::max(maxVel, 0.65);
    if (targetVelocity.length() > maxVel) targetVelocity = targetVelocity.stretchToLength(maxVel);

    command.velocity = targetVelocity;
    command.yaw = yaw;
    command.id = robot->getId();

    // forward the generated command to the ControlModule, for checking and limiting
    forwardRobotCommand();

    // Check if successful
    double errorMargin = constants::GO_TO_POS_ANGLE_ERROR_MARGIN * M_PI;
    if (directionVector.toAngle().shortestAngleDiff(yaw) < errorMargin) {
        counter++;
    } else {
        counter = 0;
    }

    // If the robot is within the error margin for 5 consecutive ticks, return success
    return (counter > 5) ? Status::Success : Status::Running;
}

const char *Orbit::getName() { return "Orbit"; }

}  // namespace rtt::ai::stp::skill
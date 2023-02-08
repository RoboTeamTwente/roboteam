#include "stp/skills/OrbitAngular.h"

#include "stp/constants/ControlConstants.h"
#include "roboteam_utils/Print.h"

namespace rtt::ai::stp::skill {

Status OrbitAngular::onUpdate(const StpInfo &info) noexcept {
    // initialization of local variables
    Angle currentAngle = info.getRobot().value()->getAngle(); // Angle the robot is currently facing
    Angle targetAngle = (info.getPositionToShootAt().value() - info.getBall()->get()->position).toAngle(); // targetAngle the robot should have
    int direction = targetAngle.rotateDirection(currentAngle) ? -1 : 1; // Direction in which the robot should move
    double speedFactor = M_PI_2; // Speed at which the robot should orbit

    double targetAngularVelocity = direction * speedFactor; // Set the target angular velocity of the robot

    Vector2 normalVector = info.getRobot().value()->getAngle().toVector2().rotate(-direction * M_PI_2); // Vector the robot should move to
    Vector2 targetVelocity; // Absolute velocity the robot should have
    targetVelocity.x = speedFactor * normalVector.x * (stp::control_constants::BALL_RADIUS + stp::control_constants::ROBOT_RADIUS); // X-Velocity
    targetVelocity.y = speedFactor * normalVector.y * (stp::control_constants::BALL_RADIUS + stp::control_constants::ROBOT_RADIUS); // Y-velocity

    // Construct the robot command
    command.id = info.getRobot().value()->getId();
    command.velocity = targetVelocity;
    command.useAngularVelocity = true;
    command.targetAngularVelocity = targetAngularVelocity;
    command.targetAngle = targetAngle;
    command.dribblerSpeed = stp::control_constants::MAX_DRIBBLER_CMD;

    forwardRobotCommand(info.getCurrentWorld()); // Send the robot command

    // Return success if the angle difference is within the margin
    if (currentAngle.shortestAngleDiff(targetAngle) < stp::control_constants::GO_TO_POS_ANGLE_ERROR_MARGIN){
        return Status::Success;
    }
    else{
        return Status::Running;
    }
}

const char *OrbitAngular::getName() { return "OrbitAngular"; }

}  // namespace rtt::ai::stp::skill
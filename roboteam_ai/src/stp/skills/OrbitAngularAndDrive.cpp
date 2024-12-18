#include "stp/skills/OrbitAngularAndDrive.h"
#include "utilities/Constants.h"
#include "utilities/GameSettings.h"

namespace rtt::ai::stp::skill {

Status OrbitAngularAndDrive::onUpdate(const StpInfo &info) noexcept {
    if (!info.getRobot() || !info.getPositionToMoveTo()) {
        return Status::Failure;
    }

    auto robot = info.getRobot().value();
    auto currentPos = robot->getPos();
    auto targetPos = info.getPositionToMoveTo().value();
    auto currentYaw = robot->getYaw();
    
    // The direction we want to face (from StpInfo)
    Angle desiredFacingAngle = info.getYaw();

    // Calculate movement direction and distance
    Vector2 moveDirection = (targetPos - currentPos);
    double distanceToTarget = moveDirection.length();

    // Calculate angular velocity for rotation
    int direction = desiredFacingAngle.rotateDirection(currentYaw) ? -1 : 1;
    double angularSpeedFactor = std::clamp(currentYaw.shortestAngleDiff(desiredFacingAngle) * 0.8 * M_PI, 0.0, M_PI);
    double targetAngularVelocity = direction * angularSpeedFactor;

    // Check if we're moving forwards/backwards relative to where we're facing
    Vector2 robotForward = Vector2(1, 0).rotate(currentYaw);

    // Calculate forwardness (-1 to 1)
    double forwardness = robotForward.dot(moveDirection.normalize());

    // Linear interpolation between 0.5 (backward) and 1.0 (forward)
    double maxSpeed = 0.2 + (forwardness + 0.4) * 0.25;

    Vector2 targetVelocity = moveDirection.normalize() * maxSpeed;

    // Construct command
    command.id = robot->getId();
    command.velocity = targetVelocity;

    // Set yaw based on robot hub mode
    if (rtt::GameSettings::getRobotHubMode() == rtt::net::RobotHubMode::BASESTATION) {
        command.yaw = currentYaw + Angle(targetAngularVelocity * 1 / 8); // 1/6
    } else {
        command.yaw = currentYaw + Angle(targetAngularVelocity * 1 / 3.33); // 1/2.5
    }
    command.dribblerOn = true;

    forwardRobotCommand();

    // Check if we've reached target position and orientation
    double posErrorMargin = constants::GO_TO_POS_ERROR_MARGIN;
    double angleErrorMargin = constants::GO_TO_POS_ANGLE_ERROR_MARGIN * M_PI;
    
    if (distanceToTarget < posErrorMargin && 
        currentYaw.shortestAngleDiff(desiredFacingAngle) < angleErrorMargin) {
        withinMarginCount++;
        if (withinMarginCount > 0) {
            return Status::Success;
        }
    } else {
        withinMarginCount = 0;
    }
    return Status::Running;
}

const char *OrbitAngularAndDrive::getName() { return "Orbit Angular And Drive"; }

}  // namespace rtt::ai::stp::skill
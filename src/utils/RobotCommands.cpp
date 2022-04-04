#include <roboteam_utils/RobotCommands.hpp>

namespace rtt {

bool RobotCommand::operator==(const RobotCommand &other) const {
    return this->id == other.id
        && this->velocity == other.velocity
        && this->targetAngle == other.targetAngle
        && this->targetAngularVelocity == other.targetAngularVelocity
        && this->useAngularVelocity == other.useAngularVelocity
        && this->cameraAngleOfRobot == other.cameraAngleOfRobot
        && this->cameraAngleOfRobotIsSet == other.cameraAngleOfRobotIsSet
        && this->kickSpeed == other.kickSpeed
        && this->waitForBall == other.waitForBall
        && this->kickType == other.kickType
        && this->dribblerSpeed == other.dribblerSpeed
        && this->ignorePacket == other.ignorePacket;
}

} // namespace rtt
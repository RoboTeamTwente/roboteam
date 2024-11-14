#include <roboteam_utils/Format.hpp>
#include <roboteam_utils/RobotCommands.hpp>

namespace rtt {

std::string kickTypeToString(KickType type) {
    switch (type) {
        case KickType::KICK:
            return "KICK";
        case KickType::CHIP:
            return "CHIP";
        case KickType::NO_KICK:
            return "NO_KICK";
        default:
            return "UNKNOWN";
    }
}

bool RobotCommand::operator==(const RobotCommand &other) const {
    return this->id == other.id && this->position == other.position && this->yaw == other.yaw  &&
           this->cameraYawOfRobot == other.cameraYawOfRobot && this->cameraYawOfRobotIsSet == other.cameraYawOfRobotIsSet &&
           this->kickSpeed == other.kickSpeed && this->waitForBall == other.waitForBall && this->kickType == other.kickType && this->kickAtYaw == other.kickAtYaw &&
           this->dribblerOn == other.dribblerOn && this->wheelsOff == other.wheelsOff;
}

std::ostream &RobotCommand::write(std::ostream &os) const {
    return os << "{"
              << "id: " << formatString("%2i", this->id) << ", "
              << ", position: " << position
            //  << "velocity: " << this->velocity << ", "
              << "yaw: " << this->yaw << ", "
            //  << "targetAngularVel: " << formatString("%5f", this->targetAngularVelocity) << ", "
            //  << "useAngularVel: " << (this->useAngularVelocity ? " true" : "false") << ", "
              << "cameraYaw: " << this->cameraYawOfRobot << ", "
              << "cameraYawIsSet: " << (this->cameraYawOfRobotIsSet ? " true" : "false") << ", "
              << "kickSpeed: " << formatString("%5f", this->kickSpeed) << ", "
              << "waitForBall: " << (this->waitForBall ? " true" : "false") << ", "
              << "kickType: " << kickTypeToString(this->kickType) << ", "
              << "kickAtYaw: " << this->kickAtYaw << ", "
              << "dribblerOn: " << formatString("%5f", this->dribblerOn) << ", "
              << "wheelsOff: " << (this->wheelsOff ? " true" : "false") << "}";
}

std::ostream &operator<<(std::ostream &os, const RobotCommand &command) { return command.write(os); }

}  // namespace rtt
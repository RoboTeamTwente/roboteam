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
        return this->id == other.id && this->velocity == other.velocity && this->targetAngle == other.targetAngle && this->targetAngularVelocity == other.targetAngularVelocity && this->useAngularVelocity == other.useAngularVelocity && this->cameraAngleOfRobot == other.cameraAngleOfRobot && this->cameraAngleOfRobotIsSet == other.cameraAngleOfRobotIsSet && this->kickSpeed == other.kickSpeed && this->waitForBall == other.waitForBall && this->kickType == other.kickType && this->kickAtAngle == other.kickAtAngle && this->dribblerSpeed == other.dribblerSpeed && this->ignorePacket == other.ignorePacket;
    }

    std::ostream &RobotCommand::write(std::ostream &os) const {
        return os << "{"
                  << "id: " << formatString("%2i", this->id) << ", "
                  << "velocity: " << this->velocity << ", "
                  << "targetAngle: " << this->targetAngle << ", "
                  << "targetAngularVel: " << formatString("%5f", this->targetAngularVelocity) << ", "
                  << "useAngularVel: " << (this->useAngularVelocity ? " true" : "false") << ", "
                  << "cameraAngle: " << this->cameraAngleOfRobot << ", "
                  << "cameraAngleIsSet: " << (this->cameraAngleOfRobotIsSet ? " true" : "false") << ", "
                  << "kickSpeed: " << formatString("%5f", this->kickSpeed) << ", "
                  << "waitForBall: " << (this->waitForBall ? " true" : "false") << ", "
                  << "kickType: " << kickTypeToString(this->kickType) << ", "
                  << "kickAtAngle: " << this->kickAtAngle << ", "
                  << "dribblerSpeed: " << formatString("%5f", this->dribblerSpeed) << ", "
                  << "ignorePacket: " << (this->ignorePacket ? " true" : "false")
                  << "}";
    }

    std::ostream &operator<<(std::ostream &os, const RobotCommand &command) {
        return command.write(os);
    }

}  // namespace rtt
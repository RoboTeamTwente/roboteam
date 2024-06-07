#include <roboteam_utils/Robot.hpp>

namespace rtt {

bool Robot::operator==(const Robot &other) const {
    return this->id == other.id && this->team == other.team && this->position == other.position && this->velocity == other.velocity && this->yaw == other.yaw &&
           this->angularVelocity == other.angularVelocity && this->ballSensorSeesBall == other.ballSensorSeesBall && this->ballSensorIsWorking == other.ballSensorIsWorking &&
           this->dribblerSeesBall == other.dribblerSeesBall && this->dribblerOn == other.dribblerOn && this->xSensIsCalibrated == other.xSensIsCalibrated &&
           this->capacitorIsCharged == other.capacitorIsCharged && this->batteryLevel == other.batteryLevel && this->radius == other.radius && this->height == other.height &&
           this->frontWidth == other.frontWidth && this->dribblerWidth == other.dribblerWidth && this->capOffset == other.capOffset;
}

}  // namespace rtt
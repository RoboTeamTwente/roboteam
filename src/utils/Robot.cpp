#include <roboteam_utils/Robot.hpp>

namespace rtt {

bool Robot::operator==(const Robot &other) const {
    return this->id == other.id
        && this->team == other.team
        && this->position == other.position
        && this->velocity == other.velocity
        && this->angle == other.angle
        && this->angularVelocity == other.angularVelocity
        && this->ballSensorSeesBall == other.ballSensorSeesBall
        && this->ballSensorIsWorking == other.ballSensorIsWorking
        && this->ballPositionOnSensor == other.ballPositionOnSensor
        && this->dribblerSeesBall == other.dribblerSeesBall
        && this->dribblerSpeed == other.dribblerSpeed
        && this->xSensIsCalibrated == other.xSensIsCalibrated
        && this->capacitorIsCharged == other.capacitorIsCharged
        && this->signalStrength == other.signalStrength
        && this->batteryLevel == other.batteryLevel
        && this->radius == other.radius
        && this->height == other.height
        && this->frontWidth == other.frontWidth
        && this->dribblerWidth == other.dribblerWidth
        && this->capOffset == other.capOffset;
}

} // namespace rtt
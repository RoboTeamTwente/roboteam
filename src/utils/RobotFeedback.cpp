#include <roboteam_utils/RobotFeedback.hpp>

namespace rtt {

bool RobotFeedback::operator==(const RobotFeedback &other) const {
    return this->id == other.id
        && this->hasBall == other.hasBall
        && this->ballPosition == other.ballPosition
        && this->ballSensorIsWorking == other.ballSensorIsWorking
        && this->velocity == other.velocity
        && this->angle == other.angle
        && this->xSensIsCalibrated == other.xSensIsCalibrated
        && this->capacitorIsCharged == other.capacitorIsCharged
        && this->wheelLocked == other.wheelLocked
        && this->wheelBraking == other.wheelBraking
        && this->batteryLevel == other.batteryLevel
        && this->signalStrength == other.signalStrength;
}

bool RobotsFeedback::operator==(const RobotsFeedback &other) const {
    return this->team == other.team
        && this->source == other.source
        && this->feedback == other.feedback;
}

} // namespace rtt
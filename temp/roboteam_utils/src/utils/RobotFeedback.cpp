#include <roboteam_utils/RobotFeedback.hpp>
#include <roboteam_utils/Format.hpp>

namespace rtt {

std::string robotFeedbackSourceToString(RobotFeedbackSource source) {
    switch (source) {
        case RobotFeedbackSource::SIMULATOR:
            return "SIMULATOR";
        case RobotFeedbackSource::BASESTATION:
            return "BASESTATION";
        default:
            return "UNKNOWN";
    }
}

bool RobotFeedback::operator==(const RobotFeedback &other) const {
    return this->id == other.id
        && this->ballSensorSeesBall == other.ballSensorSeesBall
        && this->ballPosition == other.ballPosition
        && this->ballSensorIsWorking == other.ballSensorIsWorking
        && this->dribblerSeesBall == other.dribblerSeesBall
        && this->velocity == other.velocity
        && this->angle == other.angle
        && this->xSensIsCalibrated == other.xSensIsCalibrated
        && this->capacitorIsCharged == other.capacitorIsCharged
        && this->wheelLocked == other.wheelLocked
        && this->wheelBraking == other.wheelBraking
        && this->batteryLevel == other.batteryLevel
        && this->signalStrength == other.signalStrength;
}

std::string boolToString(bool b) {
    return (b ? " true" : "false");
}

std::ostream &RobotFeedback::write(std::ostream &os) const {
    return os << "{"
        << "id: " << formatString("%2i", this->id) << ", "
        << "ballSensorSeesBall: " << boolToString(this->ballSensorSeesBall) << ", "
        << "ballPos: " << formatString("%5f", this->ballPosition) << ", "
        << "ballSensorWorks: " << boolToString(this->ballSensorIsWorking) << ", "
        << "dribblerSeesBall: " << boolToString(this->dribblerSeesBall) << ", "
        << "velocity: " << this->velocity << ", "
        << "angle: " << this->angle << ", "
        << "xSensCalib: " << boolToString(this->xSensIsCalibrated) << ", "
        << "capacitorCharged: " << boolToString(this->capacitorIsCharged) << ", "
        << "wheelLocked: " << formatString("%4i", this->wheelLocked) << ", "
        << "wheelBraking: " << formatString("%4i", this->wheelBraking) << ", "
        << "battery: " << formatString("%4f", this->batteryLevel) << ", "
        << "signalStrength: " << formatString("%3i", this->signalStrength)
        << "}";
}

std::ostream &operator<<(std::ostream &os, const RobotFeedback& feedback) {
    return feedback.write(os);
}

bool RobotsFeedback::operator==(const RobotsFeedback &other) const {
    return this->team == other.team
        && this->source == other.source
        && this->feedback == other.feedback;
}

std::ostream &RobotsFeedback::write(std::ostream &os) const {
    os << "{"
        << "Team: " << teamToString(this->team) << ", "
        << "Source: " << robotFeedbackSourceToString(this->source) << ", "
        << "Robots: [" << std::endl;
    for (const auto& _feedback : this->feedback) {
        os << " - " << _feedback << ", " << std::endl;
    }
    return os << "]}";
}

std::ostream &operator<<(std::ostream &os, const RobotsFeedback& feedbacks) {
    return feedbacks.write(os);
}

} // namespace rtt
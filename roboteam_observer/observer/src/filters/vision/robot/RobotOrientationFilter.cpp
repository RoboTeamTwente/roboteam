#include "filters/vision/robot/RobotOrientationFilter.h"
RobotOrientationFilter::RobotOrientationFilter(const Eigen::Vector2d &initialState, const Eigen::Matrix2d &initialCovariance, double modelError, double measurementError,
                                               const Time &timeStamp)
    : PosVelFilter1D(initialState, initialCovariance, modelError, measurementError, timeStamp) {}

double RobotOrientationFilter::limitAngle(double yaw) {
    yaw += M_PI;
    yaw = fmod(yaw, 2 * M_PI);
    yaw -= M_PI;

    return yaw;
}

void RobotOrientationFilter::update(const double &position) {
    // We need to do something about the rotation's discontinuities at -pi/pi so it works correctly.
    // We allow the state to go outside of bounds [-PI,PI) in between updates, but then simply make sure the observation difference is correct
    if (position < -0.5 * M_PI && lastObservation > 0.5 * M_PI) {
        orientationTurns++;
    }
    if (position > 0.5 * M_PI && lastObservation < -0.5 * M_PI) {
        orientationTurns--;
    }
    lastObservation = position;
    double correctedObservation = position + 2 * M_PI * orientationTurns;
    PosVelFilter1D::update(correctedObservation);
}

double RobotOrientationFilter::getPosition() const { return limitAngle(PosVelFilter1D::getPosition()); }
double RobotOrientationFilter::getPositionEstimate(const Time &time) const { return limitAngle(PosVelFilter1D::getPositionEstimate(time)); }
//
// Created by rolf on 24-06-21.
//

#include "filters/vision/robot/RobotOrientationFilter.h"
RobotOrientationFilter::RobotOrientationFilter(const Eigen::Vector2d &initialState,
                                               const Eigen::Matrix2d &initialCovariance,
                                               double modelError,
                                               double measurementError,
                                               const Time &timeStamp) :
    PosVelFilter1D(initialState,initialCovariance,modelError,measurementError,timeStamp){}

double RobotOrientationFilter::limitAngle(double angle) {
  //TODO: use mod and a non-while loop
  while (angle >= M_PI) {
    angle -= 2 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2 * M_PI;
  }
  return angle;
}

void RobotOrientationFilter::update(const double &position) {
  // We need to do something about the rotation's discontinuities at -pi/pi so it works correctly.
  // We allow the state to go outside of bounds [-PI,PI) in between updates, but then simply make sure the observation difference is correct
  double stateRot = filter.state()[0];
  double limitedRot = limitAngle(stateRot);
  if (stateRot != limitedRot) {
    setPosition(limitedRot);  // We're adjusting the value of the Kalman Filter here, be careful.
    // We're only doing this so we don't get flips from -inf to inf and loss of double precision at high values.
  }
  double difference = limitAngle(position - stateRot);
  double correctedObservation = limitedRot + difference;
  PosVelFilter1D::update(correctedObservation);
}

double RobotOrientationFilter::getPosition() const {
  return limitAngle(PosVelFilter1D::getPosition());
}
double RobotOrientationFilter::getPositionEstimate(const Time &time) const {
  return limitAngle(PosVelFilter1D::getPositionEstimate(time));
}
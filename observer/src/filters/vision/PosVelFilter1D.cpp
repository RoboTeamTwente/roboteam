//
// Created by rolf on 24-06-21.
//

#include "filters/vision/PosVelFilter1D.h"

bool PosVelFilter1D::predict(const Time &timeStamp) {

  double dt = (timeStamp-lastUpdateTime).asSeconds();
  if (dt<0){
    return dt == 0.0;
  }
  lastUpdateTime = timeStamp;
  setTransitionMatrix(dt);
  setProcessNoiseFromRandomAcceleration(dt);
  filter.predict();
  return true;
}
PosVelFilter1D::PosVelFilter1D(const Eigen::Vector2d &initialState,
                               const Eigen::Matrix2d &initialCovariance,
                               double modelError,
                               double measurementError,
                               const Time &timeStamp) :
    filter{initialState,initialCovariance},
    lastUpdateTime{timeStamp},
    modelError{modelError}{
  filter.R(0,0) = measurementError;
}
void PosVelFilter1D::update(const double &position) {
  filter.update(Eigen::Matrix<double,1,1>(position));
}
double PosVelFilter1D::getPositionUncertainty() const{
  return sqrt(filter.covariance()(0,0));
}
double PosVelFilter1D::getVelocityUncertainty() const{
  return sqrt(filter.covariance()(1,1));
}
void PosVelFilter1D::setMeasurementError(double error) {
  filter.R(0,0) = error;
}
void PosVelFilter1D::setTransitionMatrix(double dt) {
  //we assume that the filter is already set to the identity.
  filter.F(0,1) = dt;
}

//http://www.robots.ox.ac.uk/~ian/Teaching/Estimation/LectureNotes2.pdf
// See page 13 ish for more explanation
void PosVelFilter1D::setProcessNoiseFromRandomAcceleration(double dt) {
  double sigmaSq = modelError*modelError;

  //Random gaussian distributed with variance sigmaSq
  double dt3 = (1.0 / 3.0) * dt * dt * dt * sigmaSq;
  double dt2 = (1.0 / 2.0) * dt * dt * sigmaSq;
  double dt1 = dt * sigmaSq;

  filter.Q(0, 0) = dt3;
  filter.Q(0, 1) = dt2;
  filter.Q(1, 0) = dt2;
  filter.Q(1, 1) = dt1;
}
const Eigen::Vector2d &PosVelFilter1D::getState() const {
  return filter.state();
}
double PosVelFilter1D::getPosition() const {
  return filter.state()[0];
}
double PosVelFilter1D::getVelocity() const {
  return filter.state()[1];
}
void PosVelFilter1D::setState(const Eigen::Vector2d &state) {
  filter.setState(state);
}
void PosVelFilter1D::setPosition(const double &position) {
  filter.modifyState(0,position);
}
void PosVelFilter1D::setVelocity(const double &velocity) {
  filter.modifyState(1,velocity);
}
double PosVelFilter1D::getPositionEstimate(const Time &time) const{
  //If query is trivial or somehow is bad, just return the latest estimate.
  if(time <= lastUpdateTime){
    return getPosition();
  }
  //return linear extrapolation of current state.
  return getPosition()+(time-lastUpdateTime).asSeconds()*getVelocity();

}
void PosVelFilter1D::setCovariance(const Eigen::Matrix2d &covariance) {
  filter.setCovariance(covariance);
}
[[maybe_unused]] Time PosVelFilter1D::lastUpdated() const {
  return lastUpdateTime;
}
Eigen::Matrix2d PosVelFilter1D::getCovariance() const {
  return filter.covariance();
}
double PosVelFilter1D::getInnovation() const {
  return filter.y(0,0);
}

//
// Created by rolf on 24-06-21.
//

#include "filters/vision/PosVelFilter2D.h"

bool PosVelFilter2D::predict(const Time &timeStamp) {

  double dt = (timeStamp-lastUpdateTime).asSeconds();
  if (dt<=0){
    return dt == 0.0;
  }
  lastUpdateTime = timeStamp;
  setTransitionMatrix(dt);
  setProcessNoiseFromRandomAcceleration(dt);
  filter.predict();

  return true;
}
PosVelFilter2D::PosVelFilter2D(const Eigen::Vector4d &initialState,
                               const Eigen::Matrix4d &initialCovariance,
                               double modelError,
                               double measurementError,
                               const Time &timeStamp) :
    filter{initialState,initialCovariance},
    lastUpdateTime{timeStamp},
    modelError{modelError}{

  filter.R(0,0) = measurementError;
  filter.R(1,1) = measurementError;
}
void PosVelFilter2D::update(const Eigen::Vector2d &position) {
  filter.update(position);
}
Eigen::Vector2d PosVelFilter2D::getPositionUncertainty() const{
  return filter.covariance().diagonal().head<2>().array().sqrt(); //Indices 0,0 and 1,1. This is optimzied by Eigen better (everything is templated/inlined)
}
Eigen::Vector2d PosVelFilter2D::getVelocityUncertainty() const{
  return filter.covariance().diagonal().tail<2>().array().sqrt(); //Indices 2,2 and 3,3. This is optimized better by Eigen3
}
void PosVelFilter2D::setMeasurementError(double error) {
  filter.R(0,0) = error;
  filter.R(0,1) = 0;
  filter.R(1,0) = 0;
  filter.R(1,1) = error;


}
void PosVelFilter2D::setTransitionMatrix(double dt) {
  //we assume that the filter is already set to the identity.
  filter.F(0,2) = dt;
  filter.F(1,3) = dt;
}

//http://www.robots.ox.ac.uk/~ian/Teaching/Estimation/LectureNotes2.pdf
// See page 13 ish for more explanation
void PosVelFilter2D::setProcessNoiseFromRandomAcceleration(double dt) {
  double sigma = modelError; //TODO: this can be any function that we want it to be?
  double dt3 = (1.0 / 3.0) * dt * dt * dt * sigma * sigma;
  double dt2 = (1.0 / 2.0) * dt * dt * sigma * sigma;
  double dt1 = dt * sigma * sigma;

  filter.Q(0,0) = dt3;
  filter.Q(0,2) = dt2;
  filter.Q(1,1) = dt3;
  filter.Q(1,3) = dt2;

  filter.Q(2,0) = dt2;
  filter.Q(2,2) = dt1;
  filter.Q(3,1) = dt2;
  filter.Q(3,3) = dt1;
}
const Eigen::Vector4d &PosVelFilter2D::getState() const {
  return filter.state();
}
Eigen::Vector2d PosVelFilter2D::getPosition() const {
  return filter.state().topLeftCorner<2,1>();
}
Eigen::Vector2d PosVelFilter2D::getVelocity() const {
  return filter.state().bottomRightCorner<2,1>();
}
void PosVelFilter2D::setState(const Eigen::Vector4d &state) {
  filter.setState(state);
}
void PosVelFilter2D::setPosition(const Eigen::Vector2d &position) {
  filter.modifyState(0,position.x());
  filter.modifyState(1,position.y());
}
void PosVelFilter2D::setVelocity(const Eigen::Vector2d &velocity) {
  filter.modifyState(2,velocity.x());
  filter.modifyState(3,velocity.y());
}
Eigen::Vector2d PosVelFilter2D::getPositionEstimate(const Time &time) const{
  //If query is trivial or somehow is bad, just return the current position
  if(time <= lastUpdateTime){
    return getPosition();
  }
  //return linear extrapolation of current state.
  return getPosition()+(time-lastUpdateTime).asSeconds()*getVelocity();

}
void PosVelFilter2D::setCovariance(const Eigen::Matrix4d &covariance) {
  filter.setCovariance(covariance);
}
Time PosVelFilter2D::lastUpdated() const {
  return lastUpdateTime;
}
Eigen::Matrix4d PosVelFilter2D::getCovariance() const {
  return filter.covariance();
}
Eigen::Vector2d PosVelFilter2D::getInnovation() const {
  return filter.y;
}
void PosVelFilter2D::addUncertainty(double posUncertainty, double velUncertainty) {
  Eigen::Matrix4d covariance = filter.covariance();
  covariance(0,0)+=posUncertainty;
  covariance(1,1)+=posUncertainty;
  covariance(2,2)+=velUncertainty;
  covariance(3,3)+=velUncertainty;
  filter.setCovariance(covariance);
}
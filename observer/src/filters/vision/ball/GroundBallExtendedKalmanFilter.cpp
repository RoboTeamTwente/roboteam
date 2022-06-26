//
// Created by rolf on 29-06-21.
//

#include "filters/vision/ball/GroundBallExtendedKalmanFilter.h"

#include <utility>
void GroundBallExtendedKalmanFilter::update(const Eigen::Vector2d &observation) {
  // Compute innovation (error between measurement and state)
  y = observation - (H * X);
  // Variance of innovation
  Eigen::Matrix2d S = H * P * H.transpose() + R;
  // compute kalman gain. For small matrices, Eigen's inverse function is efficient
  Eigen::Matrix<double, 4, 2> K = P * H.transpose() * S.inverse();
  // update state with prediction
  X = X + K * y;
  // update covariance
  P -= K * H * P;
}

GroundBallExtendedKalmanFilter::GroundBallExtendedKalmanFilter(Eigen::Vector4d   initialState, Eigen::Matrix4d  initialCovariance,
                             double modelError, double measurementError, Time timeStamp) :
    modelError{modelError},
    lastUpdateTime{timeStamp},
    X{std::move(initialState)},
    P{std::move(initialCovariance)},
    F{Eigen::Matrix4d::Identity()},
    H{Eigen::Matrix<double, 2, 4>::Identity()},
    Q{Eigen::Matrix4d::Zero()},
    R{Eigen::Matrix2d::Identity() * measurementError},
    y{Eigen::Vector2d::Zero()} {}

Eigen::Vector2d GroundBallExtendedKalmanFilter::getPosition() const {
  return X.head<2>();
}

Eigen::Vector2d GroundBallExtendedKalmanFilter::getVelocityUncertainty() const {
  return P.diagonal().tail<2>().array().sqrt();
}

Eigen::Vector2d GroundBallExtendedKalmanFilter::getVelocity() const {
  return X.tail<2>();
}

void GroundBallExtendedKalmanFilter::setVelocity(const Eigen::Vector2d &velocity) {
  X.tail<2>() = velocity;
}

void GroundBallExtendedKalmanFilter::addUncertainty(double posUncertainty, double velUncertainty) {
  P.diagonal().head<2>().array() += posUncertainty;
  P.diagonal().tail<2>().array() += velUncertainty;
}

Eigen::Vector2d GroundBallExtendedKalmanFilter::getPositionUncertainty() const {
  return P.diagonal().head<2>().array().sqrt();
}

double GroundBallExtendedKalmanFilter::getAcceleration() const {
  return acceleration;
}

void GroundBallExtendedKalmanFilter::setAcceleration(double accel) {
  acceleration = accel;
  assert(acceleration != 0.0);//This class isn't designed to work when acceleration is set to exactly 0.0
}

void GroundBallExtendedKalmanFilter::setProccessNoise(double dt) {
  double sigma = modelError;
  double dt3 = (1.0 / 3.0) * dt * dt * dt * sigma * sigma;
  double dt2 = (1.0 / 2.0) * dt * dt * sigma * sigma;
  double dt1 = dt * sigma * sigma;

  Q(0, 0) = dt3;
  Q(0, 2) = dt2;
  Q(1, 1) = dt3;
  Q(1, 3) = dt2;

  Q(2, 0) = dt2;
  Q(2, 2) = dt1;
  Q(3, 1) = dt2;
  Q(3, 3) = dt1;
}

Eigen::Vector4d GroundBallExtendedKalmanFilter::getStateEstimate(double dt) const {
  Eigen::Vector2d currentPos = getPosition();
  Eigen::Vector2d currentVel = getVelocity();
  double vel = currentVel.norm();
  Eigen::Vector4d estimate;
  if (vel > BALL_STILL_VELOCITY) {
    estimate.head<2>() = currentPos + currentVel * dt + 0.5 * currentVel / vel * acceleration * dt * dt;
    estimate.tail<2>() = currentVel + currentVel / vel * acceleration * dt;
  } else {
    estimate.head<2>() = currentPos;
    estimate.tail<2>() = Eigen::Vector2d::Zero();
  }

  return estimate;
}

Eigen::Vector4d GroundBallExtendedKalmanFilter::getStateEstimate(const Time &time) const {
  double frame_dt = (time - lastUpdateTime).asSeconds();
  //if the update is from now or in the past, just return the current state estimate; we don't extrapolate to the past
  if (frame_dt <= 0) {
    return X;
  }
  double vel = getVelocity().norm();
  //We need to check if the velocity does not reach 0, as at that point the ball simply lays still
  //Because of the dimples it usually lays still when it reaches ~= 0.01 cm/s but this is typically negligible
  double dt = frame_dt;
  if (vel + acceleration * frame_dt < 0) {
    dt = -vel / acceleration;
  }
  return getStateEstimate(dt);
}

Eigen::Vector2d GroundBallExtendedKalmanFilter::getVelocityEstimate(const Time &time) const {
  return getStateEstimate(time).tail<2>();
}

Eigen::Vector2d GroundBallExtendedKalmanFilter::getPositionEstimate(const Time &time) const {
  return getStateEstimate(time).head<2>();
}

Eigen::Vector2d GroundBallExtendedKalmanFilter::innovation() const {
  return y;
}

Time GroundBallExtendedKalmanFilter::lastUpdated() const {
  return lastUpdateTime;
}

Eigen::Vector4d GroundBallExtendedKalmanFilter::state() const {
  return X;
}

Eigen::Matrix4d GroundBallExtendedKalmanFilter::covariance() const {
  return P;
}



void GroundBallExtendedKalmanFilter::predict(Time timeStamp) {

  assert(timeStamp>= lastUpdateTime);
  double frame_dt = (timeStamp - lastUpdateTime).asSeconds();
  if (frame_dt <= 0) {
    return;
  }
  lastUpdateTime = timeStamp;

  Eigen::Vector2d currentPos = getPosition();
  Eigen::Vector2d currentVel = getVelocity();
  double vel = currentVel.norm();
  //We need to check if the velocity does not reach 0, as at that point the ball simply lays still. Otherwise, we accelerate the ball 'backwards'
  //Because of the dimples it usually lays still when it reaches ~= 0.01 cm/s but we ignore this in our model
  double dt = frame_dt;
  if (vel + acceleration * frame_dt < 0) {
    dt = -vel / acceleration;
  }

  //update the transition matrix
  double velCubed = vel * vel * vel;
  double vysq = currentVel.y() * currentVel.y() / velCubed;
  double vxvy = -currentVel.x() * currentVel.y() / velCubed;
  double vxsq = currentVel.x() * currentVel.x() / velCubed;

  //To prevent division by zero, we simply put the velocity of the ball to zero if the ball is almost lying still
  if (vel <= BALL_STILL_VELOCITY) {
    vxsq = 0.0;
    vxvy = 0.0;
    vysq = 0.0;
  }

  F(0, 2) = dt + 0.5 * vysq * acceleration * dt * dt;
  F(0, 3) = 0.5 * vxvy * acceleration * dt * dt;
  F(1, 2) = 0.5 * vxvy * acceleration * dt * dt;
  F(1, 3) = dt + 0.5 * vxsq * acceleration * dt * dt;
  F(2, 2) = 1 + vysq * acceleration * dt;
  F(2, 3) = vxvy * acceleration * dt;
  F(3, 2) = vxvy * acceleration * dt;
  F(3, 3) = 1 + vxsq * acceleration * dt;

  //setting the process noise matrix (Q)
  setProccessNoise(frame_dt);

  //now update the state given the transition function
  if (vel > BALL_STILL_VELOCITY) {
    X.head<2>() = currentPos + currentVel * dt + 0.5 * currentVel / vel * acceleration * dt * dt;
    X.tail<2>() = currentVel + currentVel / vel * acceleration * dt;
  }else{ //Set the velocity of the ball to zero
      X.tail<2>() = Eigen::Vector2d::Zero();
  }
  P = F * P * F.transpose() + Q;

}
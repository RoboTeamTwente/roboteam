//
// Created by rolf on 23-06-21.
//

#include <filters/vision/ball/CameraGroundBallFilter.h>

#include <utility>

void CameraGroundBallFilter::update(const BallObservation &observation) {
  ekf.update(observation.position);
  objectSeen(observation.timeCaptured);
}
bool CameraGroundBallFilter::updateNotSeen(Time time) {
  objectInvisible(time);
  return getHealth() <= 0.0 && consecutiveFramesNotSeen() >= 3;
}
bool CameraGroundBallFilter::processDetections(const CameraGroundBallPredictionObservationPair &prediction_observation_pair) {
  predictFilter(prediction_observation_pair.prediction);
  bool removeFilter = false;
  if (prediction_observation_pair.observation.has_value()) {
    update(prediction_observation_pair.observation.value());
  } else {
    removeFilter = updateNotSeen(prediction_observation_pair.prediction.time);
  }
  return removeFilter;
}
void CameraGroundBallFilter::predictFilter(const CameraGroundBallPrediction &prediction) {
  //simple function for now but may become complicated with collisions
  ekf.predict(prediction.time);
}
Eigen::Vector2d CameraGroundBallFilter::getVelocityEstimate(Time time) const {
  return ekf.getVelocityEstimate(time);
}
CameraGroundBallFilter::CameraGroundBallFilter(const BallObservation &observation,
                                               const Eigen::Vector2d &velocity_estimate) :
    CameraObjectFilter(0.2, 1 / 60.0, 15, 3, observation.timeCaptured) {
  Eigen::Vector4d startState = {observation.position.x(),observation.position.y(),velocity_estimate.x(),velocity_estimate.y()};
  Eigen::Matrix4d startCovariance = Eigen::Matrix4d::Zero();
  constexpr double BALL_POSITION_INITIAL_COV = 0.05;//[m] uncertainty in initial ball position
  constexpr double BALL_VELOCITY_INITIAL_COV = 4.0; //[m/s]

  startCovariance(0, 0) = BALL_POSITION_INITIAL_COV;
  startCovariance(1, 1) = BALL_POSITION_INITIAL_COV;
  startCovariance(2, 2) = BALL_VELOCITY_INITIAL_COV;
  startCovariance(3, 3) = BALL_VELOCITY_INITIAL_COV;

  constexpr double BALL_MODEL_ERROR = 1.0;
  constexpr double BALL_MEASUREMENT_ERROR = 0.002; //[m] estimated average position uncertainty in ball detections
  ekf = GroundBallExtendedKalmanFilter(startState,startCovariance,BALL_MODEL_ERROR,BALL_MEASUREMENT_ERROR,observation.timeCaptured);
}
CameraGroundBallPrediction CameraGroundBallFilter::predict(Time time) const {
  const auto& estimate = ekf.getStateEstimate(time);
  CameraGroundBallPrediction prediction(estimate.head<2>(),estimate.tail<2>(),time);
  return prediction;
}
FilteredBall CameraGroundBallFilter::getEstimate(Time time) const {
  FilteredBall ball(ekf.getPositionEstimate(time),
                    ekf.getVelocityEstimate(time),
                    getHealth(),
                    ekf.getPositionUncertainty().norm(),
                    ekf.getVelocityUncertainty().norm());
  return ball;
}

CameraGroundBallPrediction::CameraGroundBallPrediction(Eigen::Vector2d pos, Eigen::Vector2d vel, Time time) :
position{std::move(pos)},velocity{std::move(vel)},time{time}{

}

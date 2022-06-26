//
// Created by rolf on 29-06-21.
//

#ifndef RTT_ROBOTEAM_WORLD_OBSERVER_SRC_FILTERS_VISION_BALL_GROUNDBALLEXTENDEDKALMANFILTER_H_
#define RTT_ROBOTEAM_WORLD_OBSERVER_SRC_FILTERS_VISION_BALL_GROUNDBALLEXTENDEDKALMANFILTER_H_

#include <Eigen/Dense>
#include <roboteam_utils/Time.h>

/**
 * @brief An extended kalman filter to keep track of a single ball.
 * The model it uses takes the current position and velocity of the ball and applies a constant acceleration in the
 * direction of travel of the ball (e.g. a rolling object slowing down with constant deceleration).
 * Here, this is implemented using an extended kalman filter
 * For ease of use, the interface of this class is highly similar to PosVelFilter2D.
 * This class is only responsible for the kalman filter, not for observation management.
 */
class GroundBallExtendedKalmanFilter {
 public:
  GroundBallExtendedKalmanFilter() = default; //only used for convenience, should probably not exist
  GroundBallExtendedKalmanFilter(Eigen::Vector4d initialState, Eigen::Matrix4d initialCovariance,
                                 double modelError, double measurementError, Time timeStamp);
  void predict(Time predictionTime);
  void update(const Eigen::Vector2d& observation);
  [[nodiscard]] Eigen::Vector2d getPosition() const;
  [[nodiscard]] Eigen::Vector2d getPositionEstimate(const Time& time) const;
  [[nodiscard]] Eigen::Vector4d getStateEstimate(const Time& time) const;
  [[nodiscard]] Eigen::Vector2d getVelocity() const;
  [[nodiscard]] Eigen::Vector2d getVelocityEstimate(const Time& time) const;

  void setVelocity(const Eigen::Vector2d& velocity);
  void addUncertainty(double posUncertainty, double velUncertainty);

  [[nodiscard]] Eigen::Vector2d getVelocityUncertainty() const;
  [[nodiscard]] Eigen::Vector2d getPositionUncertainty() const;

  [[nodiscard]] double getAcceleration() const;
  void setAcceleration(double accel);

  [[nodiscard]] Eigen::Vector2d innovation() const;
  [[nodiscard]] Time lastUpdated() const;

  [[nodiscard]] Eigen::Vector4d state() const;
  [[nodiscard]] Eigen::Matrix4d covariance() const;
 private:
  void setProccessNoise(double dt);
  [[nodiscard]] Eigen::Vector4d getStateEstimate(double dt) const;

  double modelError = 0.0;
  double acceleration = -0.3; //TODO: make adjustable/fix
  static constexpr double BALL_STILL_VELOCITY = 0.01;//[m/s]. This is somewhat realistic as due to the dimples in the ball, the ball also stops at roughly this velocity
  Time lastUpdateTime;
  Eigen::Vector4d X;
  Eigen::Matrix4d P;
  Eigen::Matrix4d F; // Forward model/state update matrix
  Eigen::Matrix<double, 2, 4> H; // Observation model/ states how we can interpret observation as our state
  Eigen::Matrix4d Q; // Covariance of the process noise. (Amount of "Random Forces" we can expect in the process)
  Eigen::Matrix2d R; // Observation Noise Covariance. Keeps track of how noisy the observations are.
  Eigen::Vector2d y; //Innovation. Not strictly necessary to store but often used to measure performance of the filter
};

#endif //RTT_ROBOTEAM_WORLD_OBSERVER_SRC_FILTERS_VISION_BALL_GROUNDBALLEXTENDEDKALMANFILTER_H_

//
// Created by rolf on 24-06-21.
//

#ifndef RTT_ROBOTEAM_WORLD_OBSERVER_INCLUDE_OBSERVER_FILTERS_VISION_POSVELFILTER2D_H_
#define RTT_ROBOTEAM_WORLD_OBSERVER_INCLUDE_OBSERVER_FILTERS_VISION_POSVELFILTER2D_H_
#include "KalmanFilter.h"
#include "roboteam_utils/Time.h"

/**
 * @brief Class that filters 2d positions. This is essentially a constant velocity kalman filter in 2 dimensions.
 * objects are assumed to keep moving as they do.
 * the first two state index represent the x and y coordinates,
 * the 3rd and fourth represent the x and y velocity respectively.
 */
class PosVelFilter2D {
 public:
  PosVelFilter2D() = default;
  PosVelFilter2D(const Eigen::Vector4d &initialState, const Eigen::Matrix4d& initialCovariance,
                 double modelError, double measurementError, const Time& timeStamp);
  /**
   * @brief Predict the filter estimate to a new timestamp when there is no new measurement.
   * Note this permanently alters the state of the filter, so there is no way to go back if you receive vision frames still.
   * @param timeStamp
   * @return false if the time to which one attempts to predict is in the past. In this case, the filter is not updated
   */
  bool predict(const Time& timeStamp);
  /**
   * @brief Update the position estimate with a new measurement at the current time of the filter.
   * @param position to update the filter with
   */
  void update(const Eigen::Vector2d &position);

  /**
   * @brief Returns the state of the filter. First two indices are position, last two are velocity
   * @return state.
   */
  [[nodiscard]] const Eigen::Vector4d& getState() const;
  /**
   * Returns the position of the filter. Prefer using getState() for performance
   * @return
   */
  [[nodiscard]] Eigen::Vector2d getPosition() const;
  /**
   * Returns the velocity of the current state of the filter. Prefer using getState() for performance
   * @return
   */
  [[nodiscard]] Eigen::Vector2d getVelocity() const;
  /**
   * Returns a linear extrapolation of the filter state to obtain position at a future time.
   * @return pos + vel*dt (essentially)
   */
  [[nodiscard]] Eigen::Vector2d getPositionEstimate(const Time &time) const;

  /**
   * @brief Gets the uncertainty in position of the current filter state
   * @return
   */
  [[nodiscard]] Eigen::Vector2d getPositionUncertainty() const;
  /**
   * @brief Gets the uncertainty in velocity of the current filter state
   * @return
   */
  [[nodiscard]] Eigen::Vector2d getVelocityUncertainty() const;
  /**
   * Sets the measurement error (R matrix)
   * @param error
   */
  void setMeasurementError(double error);

  //The following functions alter the state of the filter. Use with care!
  /**
   * Resets the state of the filter to the given state
   * @param state
   */
  void setState(const Eigen::Vector4d &state);
  /**
   * Resets the position of the filter to the given position
   * @param position
   */
  void setPosition(const Eigen::Vector2d &position);
  /**
   * Resets the velocity of the filter to the given position
   * @param velocity
   */
  void setVelocity(const Eigen::Vector2d &velocity);

  /**
   * Resets the covariance of the filter to the given matrix.
   * @param covariance
   */
  void setCovariance(const Eigen::Matrix4d &covariance);

  /**
   * @return the time the filter was last predicted to.
   */
  [[nodiscard]] Time lastUpdated() const;

  /**
   * @return The current covariance of the kalman filter
   */
  [[nodiscard]] Eigen::Matrix4d getCovariance() const;

  /**
   * @return the innovation, e.g. the difference between the last prediction and observation in the last update function.
   */
  [[nodiscard]] Eigen::Vector2d getInnovation() const;

  /**
   * Adds uncertainty to the filter by increasing the diagonal entries of the covariance matrix
   * @param posUncertainty
   * @param velUncertainty
   */
  void addUncertainty(double posUncertainty, double velUncertainty);
 private:
  //Before every tick we need to set the matrices we use using the dt of the tick
  void setTransitionMatrix(double dt);
  void setProcessNoiseFromRandomAcceleration(double dt);
  KalmanFilter<4,2> filter;
  Time lastUpdateTime;
  double modelError = 0.0;

};

#endif //RTT_ROBOTEAM_WORLD_OBSERVER_INCLUDE_OBSERVER_FILTERS_VISION_POSVELFILTER2D_H_

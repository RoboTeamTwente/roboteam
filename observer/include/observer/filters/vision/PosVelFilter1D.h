//
// Created by rolf on 24-06-21.
//

#ifndef RTT_ROBOTEAM_WORLD_OBSERVER_SRC_FILTERS_VISION_POSVELFILTER1D_H_
#define RTT_ROBOTEAM_WORLD_OBSERVER_SRC_FILTERS_VISION_POSVELFILTER1D_H_

#include "KalmanFilter.h"
#include "roboteam_utils/Time.h"

class PosVelFilter1D {
 public:
  PosVelFilter1D() = default;
  PosVelFilter1D(const Eigen::Vector2d &initialState, const Eigen::Matrix2d& initialCovariance,
                 double modelError, double measurementError, const Time& timeStamp);
  /**
   * @brief Predict the filter estimate to a new timestamp when there is no new measurement.
   * Note this permanently alters the state of the filter, so there is no way to go back if you receive vision frames still.
   * @param timeStamp
   * @returns false if timestamp is from the past. In this case the filter is not predicted/changed. True otherwise
   */
  bool predict(const Time& timeStamp);
  /**
   * @brief Update the position estimate with a new measurement at the current time of the filter.
   * @param position to update the filter with
   */
  virtual void update(const double &position);

  /**
   * @brief Returns the state of the filter. First two indices are position, last two are velocity
   * @return state.
   */
  [[nodiscard]] virtual const Eigen::Vector2d& getState() const;
  /**
   * Returns the position of the filter. Prefer using getState() for performance
   * @return
   */
  [[nodiscard]] virtual double getPosition() const;
  /**
   * Returns the velocity of the current state of the filter. Prefer using getState() for performance
   * @return
   */
  [[nodiscard]] double getVelocity() const;
  /**
   * Returns a linear extrapolation of the filter state to obtain position at a future time.
   * @return pos + vel*dt (essentially)
   */
  [[nodiscard]] virtual double getPositionEstimate(const Time &time) const;

  /**
   * @brief Gets the uncertainty in position of the current filter state
   * @return
   */
  [[nodiscard]] double getPositionUncertainty() const;
  /**
   * @brief Gets the uncertainty in velocity of the current filter state
   * @return
   */
  [[nodiscard]] double getVelocityUncertainty() const;
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
  void setState(const Eigen::Vector2d &state);
  /**
   * Resets the position of the filter to the given position
   * @param position
   */
  void setPosition(const double &position);
  /**
   * Resets the velocity of the filter to the given position
   * @param velocity
   */
  void setVelocity(const double &velocity);

  /**
   * Resets the covariance of the filter to the given matrix.
   * @param covariance
   */
  void setCovariance(const Eigen::Matrix2d &covariance);

  /**
   * @return The last time the filter was predicted to.
   */
  [[nodiscard]] Time lastUpdated() const;

  /**
   * @return The covariance matrix of the filter
   */
  [[nodiscard]] Eigen::Matrix2d getCovariance() const;

  /**
   * @return the innovation, e.g. the difference between the last observation and the last prediction
   */
  [[nodiscard]] double getInnovation() const;

 protected:
  KalmanFilter<2,1> filter;
 private:
  //Before every tick we need to set the matrices we use using the dt of the tick
  void setTransitionMatrix(double dt);
  void setProcessNoiseFromRandomAcceleration(double dt);

  Time lastUpdateTime;
  double modelError = 0.0;

};

#endif //RTT_ROBOTEAM_WORLD_OBSERVER_SRC_FILTERS_VISION_POSVELFILTER1D_H_

//
// Created by rolf on 24-06-21.
//

#ifndef RTT_ROBOTEAM_WORLD_OBSERVER_SRC_FILTERS_VISION_ROBOT_ROBOTORIENTATIONFILTER_H_
#define RTT_ROBOTEAM_WORLD_OBSERVER_SRC_FILTERS_VISION_ROBOT_ROBOTORIENTATIONFILTER_H_

#include "observer/filters/vision/PosVelFilter1D.h"

/**
 * @brief A class which tracks robot angle; this is a 1 dimensional posvelFilter which applies mod 2pi, essentially
 */
class RobotOrientationFilter : public PosVelFilter1D {
 public:
  RobotOrientationFilter() : PosVelFilter1D(){}
  RobotOrientationFilter(const Eigen::Vector2d &initialState,
                         const Eigen::Matrix2d &initialCovariance,
                         double modelError,
                         double measurementError,
                         const Time &timeStamp);
  /**
   * @param kalman update, making sure the angle is safe
   */
  void update(const double& position) override;
  /**
   * @return get's the current robot angle
   */
  [[nodiscard]] double getPosition() const override;
  /**
   * @brief Gets the angle estimate at the given time
   * @param time
   * @return the estimate of the angle at the given timm
   */
  [[nodiscard]] double getPositionEstimate(const Time &time) const override;
  /**
   * @brief projects the angle from any value x to the range [-pi,pi)
   * @param angle
   * @return the limited angle
   */
  [[nodiscard]] static double limitAngle(double angle);
};

#endif //RTT_ROBOTEAM_WORLD_OBSERVER_SRC_FILTERS_VISION_ROBOT_ROBOTORIENTATIONFILTER_H_

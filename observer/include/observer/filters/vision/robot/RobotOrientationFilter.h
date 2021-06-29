//
// Created by rolf on 24-06-21.
//

#ifndef RTT_ROBOTEAM_WORLD_OBSERVER_SRC_FILTERS_VISION_ROBOT_ROBOTORIENTATIONFILTER_H_
#define RTT_ROBOTEAM_WORLD_OBSERVER_SRC_FILTERS_VISION_ROBOT_ROBOTORIENTATIONFILTER_H_

#include "observer/filters/vision/PosVelFilter1D.h"
//Note the getState() function returns invalid results for this class!
class RobotOrientationFilter : public PosVelFilter1D {
 public:
  RobotOrientationFilter() :PosVelFilter1D(){}
  RobotOrientationFilter(const Eigen::Vector2d &initialState,
                         const Eigen::Matrix2d &initialCovariance,
                         double modelError,
                         double measurementError,
                         const Time &timeStamp);
  void update(const double& position) override;
  [[nodiscard]] double getPosition() const override;
  [[nodiscard]] double getPositionEstimate(const Time &time) const override;
  [[nodiscard]] static double limitAngle(double angle);
};

#endif //RTT_ROBOTEAM_WORLD_OBSERVER_SRC_FILTERS_VISION_ROBOT_ROBOTORIENTATIONFILTER_H_

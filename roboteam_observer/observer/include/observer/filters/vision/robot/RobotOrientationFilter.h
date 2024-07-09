#ifndef RTT_ROBOTEAM_OBSERVER_OBSERVER_SRC_FILTERS_VISION_ROBOT_ROBOTORIENTATIONFILTER_H_
#define RTT_ROBOTEAM_OBSERVER_OBSERVER_SRC_FILTERS_VISION_ROBOT_ROBOTORIENTATIONFILTER_H_

#include "observer/filters/vision/PosVelFilter1D.h"

/**
 * @brief A class which tracks robot yaw; this is a 1 dimensional posvelFilter which applies mod 2pi, essentially
 */
class RobotOrientationFilter : public PosVelFilter1D {
   public:
    RobotOrientationFilter() : PosVelFilter1D() {}
    RobotOrientationFilter(const Eigen::Vector2d &initialState, const Eigen::Matrix2d &initialCovariance, double modelError, double measurementError, const Time &timeStamp);
    /**
     * @param kalman update, making sure the yaw is safe
     */
    void update(const double &position) override;
    /**
     * @return get's the current robot yaw
     */
    [[nodiscard]] double getPosition() const override;
    /**
     * @brief Gets the yaw estimate at the given time
     * @param time
     * @return the estimate of the yaw at the given timm
     */
    [[nodiscard]] double getPositionEstimate(const Time &time) const override;
    /**
     * @brief projects the yaw from any value x to the range [-pi,pi)
     * @param yaw
     * @return the limited yaw
     */
    [[nodiscard]] static double limitAngle(double yaw);
    double lastObservation = 0.0;
    int orientationTurns = 0;
};

#endif  // RTT_ROBOTEAM_OBSERVER_OBSERVER_SRC_FILTERS_VISION_ROBOT_ROBOTORIENTATIONFILTER_H_

#ifndef RTT_YAWPID_H
#define RTT_YAWPID_H

#include "roboteam_utils/Angle.h"
#include "roboteam_utils/pid.h"
namespace rtt {
/**
 * @brief Class that controls the yaw with a PID
 */
class YawPID {
   private:
    double P;              /**< Proportional part of the controller */
    double I;              /**< Integral part of the controller */
    double D;              /**< Derivative part of the controller */
    double min;            /**< Minimum allowed angular velocity */
    double max;            /**< Maximum allowed angular velocity */
    double previous_error; /**< Error from previous tick */
    double integral;       /**< Integral of the previous errors */
    double dt;             /**< Time difference since previous tick */

   public:
    /**
     * @brief Constructor of the YawPID class
     * @param P Proportional part of the controller
     * @param I Integral part of the controller
     * @param D Derivative part of the controller
     * @param max_ang_vel Absolute maximum allowed angular velocity
     * @param dt Time difference since previous tick
     */
    YawPID(double P, double I, double D, double max_ang_vel, double dt) : P{P}, I{I}, D{D}, min{-max_ang_vel}, max{max_ang_vel}, previous_error{0.0}, integral{0.0}, dt{dt} {}

    /**
     * @brief Retrieves the angular velocity calculated by the controller
     * @param target_yaw Yaw the robot needs to be at
     * @param current_yaw Current yaw of the robot
     * @return Target angular velocity
     */
    double getOutput(Angle target_yaw, Angle current_yaw);
};
}  // namespace rtt

#endif  // RTT_YAWPID_H

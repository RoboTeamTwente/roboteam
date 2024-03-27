#ifndef RTT_ANGLEPID_H
#define RTT_ANGLEPID_H

#include "roboteam_utils/Angle.h"
#include "roboteam_utils/pid.h"
namespace rtt {
/**
 * @brief Class that controls the angle with a PID
 */
class AnglePID {
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
     * @brief Constructor of the AnglePID class
     * @param P Proportional part of the controller
     * @param I Integral part of the controller
     * @param D Derivative part of the controller
     * @param max_ang_vel Absolute maximum allowed angular velocity
     * @param dt Time difference since previous tick
     */
    AnglePID(double P, double I, double D, double max_ang_vel, double dt) : P{P}, I{I}, D{D}, min{-max_ang_vel}, max{max_ang_vel}, previous_error{0.0}, integral{0.0}, dt{dt} {}

    /**
     * @brief Retrieves the angular velocity calculated by the controller
     * @param target_angle Angle the robot needs to be at
     * @param current_angle Current angle of the robot
     * @return Target angular velocity
     */
    double getOutput(Angle target_angle, Angle current_angle);
};
}  // namespace rtt

#endif  // RTT_ANGLEPID_H

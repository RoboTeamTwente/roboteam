//
// Created by ratoone on 30-01-20.
//

#ifndef RTT_POSITIONCONTROLUTILS_H
#define RTT_POSITIONCONTROLUTILS_H

#include "interface/api/Output.h"
#include "roboteam_utils/Vector2.h"
#include "utilities/StpInfoEnums.h"

namespace rtt::ai::control {

class PositionControlUtils {
   public:
    constexpr static const double TIME_STEP = 0.1;
    constexpr static const int COLLISION_DETECTOR_STEP_COUNT = 10;
    constexpr static const double MIN_ROBOT_DISTANCE = 3 * ai::Constants::ROBOT_RADIUS_MAX();

    constexpr static const double MAX_TARGET_DEVIATION = 0.05;
    constexpr static const double MIN_DISTANCE_TO_TARGET = 0.05;
    constexpr static const double MAX_STALE_VELOCITY = 0.05;


    /**
     * If the distance between the old target and the new target > MAX_TARGET_DEVIATION
     * @param targetPos
     * @param oldTarget
     * @return
     */
    static bool hasTargetChanged(const Vector2 &targetPos, const Vector2 &oldTarget);

    /**
     * The target is considered to be reached if the distance between target and currentPosition < MIN_DISTANCE_TO_TARGET
     * @param targetPos
     * @param currentPosition
     * @return
     */
    static bool isTargetReached(const Vector2 &targetPos, const Vector2 &currentPosition);

    /**
     * Is the target moving velocity > MAX_STALE_VELOCITY
     * @param velocity
     */
    static bool isMoving(const Vector2 &velocity);

    /**
     * The getPIDValue function takes a stp::PIDType enum class as input and returns a given pid value
     * @param pidType  The different stp::PIDType options correspond to different robot behaviors, such as default movement,
     *                 receiving a pass, intercepting the ball, and playing as the goalkeeper.
     * @return pidVals struct, which contains the proportional gain (kp), integral gain (ki), and derivative gain (kd) for a PID
     */
    static pidVals getPIDValue(const stp::PIDType &pidType);

    /**
     *  This function takes value in seconds and converts it to an integer value representing the number of time steps
     * @param time in seconds
     * @return number of time steps
     */
    static int convertTimeToStep(double time);

    /**
     * This function takes value in time steps and converts it to a double value representing the number of seconds
     * @param step number of time steps
     * @return time in seconds
     */
    static double convertStepToTime(int step);
};
}  // namespace rtt::ai::control

#endif  // RTT_POSITIONCONTROLUTILS_H
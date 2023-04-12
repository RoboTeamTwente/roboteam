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

    constexpr static const double MIN_ENEMY_MOVING_ROBOT_DISTANCE = 3 * ai::Constants::ROBOT_RADIUS_MAX();
    constexpr static const double MIN_ENEMY_STALE_ROBOT_DISTANCE = 2 * ai::Constants::ROBOT_RADIUS_MAX();

    constexpr static const double MIN_OUR_ROBOT_DISTANCE_MOVING = 3 * ai::Constants::ROBOT_RADIUS_MAX();

    constexpr static const double POSITION_TOLERANCE_DISTANCE = 0.05;
    constexpr static const double MAX_STALE_VELOCITY = 0.05;

    /**
     * Checks if two 2D vectors are within a certain distance tolerance.
     * @param pos1 The first 2D vector to compare.
     * @param pos2 The second 2D vector to compare.
     * @return True if the two vectors are within the distance tolerance, False otherwise.
     */
    static bool positionWithinTolerance(const Vector2 &pos1, const Vector2 &pos2);

    /**
     * Determines if the object is currently moving based on its velocity.
     *
     * @param velocity The 2D velocity of the object.
     * @return `true` if the magnitude of the velocity is greater than zero, indicating that the object is moving;
     * `false` otherwise.
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
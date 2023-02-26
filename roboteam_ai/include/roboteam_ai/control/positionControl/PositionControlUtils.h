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
    constexpr static const int MAX_ROBOTS_PER_TEAM = 11;

    /**
     * If the distance between the old target and the new target > MAX_TARGET_DEVIATION
     * @param targetPos
     * @param oldTarget
     * @return
     */
    static bool isTargetChanged(const Vector2 &targetPos, const Vector2 &oldTarget);

    /**
     * The target is considered to be reached if the distance between target and currentPosition < MIN_DISTANCE
     * @param targetPos
     * @param currentPosition
     * @return
     */
    static bool isTargetReached(const Vector2 &targetPos, const Vector2 &currentPosition);

    /**
     * Is the target moving
     * @param velocity
     */
    static bool isMoving(const Vector2 &velocity);

    static pidVals getPIDValue(const stp::PIDType &pidType);

    static int convertTimeToStep(double time);
    static double convertStepToTime(int step);
};
}  // namespace rtt::ai::control

#endif  // RTT_POSITIONCONTROLUTILS_H
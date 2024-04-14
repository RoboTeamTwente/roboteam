#ifndef RTT_BBTPATHTRACKING_H
#define RTT_BBTPATHTRACKING_H

#include "PidTracking.h"
#include "control/positionControl/PositionControlUtils.h"
#include "roboteam_utils/Position.h"
#include "roboteam_utils/Vector2.h"
#include "utilities/Constants.h"

namespace rtt::ai::control {

/**
 * @brief Path tracking algorithm for BBT
 */
class BBTPathTracking {
   private:
    static constexpr unsigned long STEPS_AHEAD = 1; /**< Maximum amount of steps that can be taken into account */
    PidTracking pidTracking;                        /**< PID tracker of the trajectory */

   public:
    /**
     * Generates an output velocity and angle according to the implemented algorithm.
     * After reaching a certain distance to the closest path point, it will go to the next one. <br><br>
     * @param currentPosition current position of the robot
     * @param currentVelocity current velocity of the robot
     * @param pathPoints the path as a list of points
     * @param velocityPoints the path as a list of velocities
     * @param robotId the ID of the current robot
     * @param angle the desired orientation angle of the robot - if omitted, the robot will face its velocity
     * @return a structure containing the tracking velocity and the orientation angle
     */
    Position trackPathForwardAngle(const Vector2 &currentPosition, const Vector2 &currentVelocity, std::vector<std::pair<Vector2, Vector2>> &pathVelocityPoints, int robotId,
                                   stp::PIDType pidType);
};

}  // namespace rtt::ai::control

#endif  // RTT_BBTPATHTRACKING_H

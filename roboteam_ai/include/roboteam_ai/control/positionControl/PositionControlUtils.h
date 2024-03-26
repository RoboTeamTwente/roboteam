#ifndef RTT_POSITIONCONTROLUTILS_H
#define RTT_POSITIONCONTROLUTILS_H

#include "roboteam_utils/Vector2.h"
#include "utilities/Constants.h"

namespace rtt::ai::control {

/**
 * @brief Class containing useful functions used in position control
 */
class PositionControlUtils {
   private:
    static constexpr double MAX_TARGET_DEVIATION = 0.05;                                 /**< Maximum distance a target can deviate before it is defined as a new position */
    static constexpr double MIN_DISTANCE_TARGET_REACHED = 2 * Constants::ROBOT_RADIUS(); /**< minimum distance needed to consider the current target reached */

   public:
    /**
     * @brief Checks whether the distance between the old target and the new target is greater than MAX_TARGET_DEVIATION
     * @param targetPos Target position to go to
     * @param oldTarget Previous target position
     * @return Boolean that tells whether the target position changed
     */
    static bool isTargetChanged(const Vector2 &targetPos, const Vector2 &oldTarget);

    /**
     * @brief checks whether the distance between target and currentPosition is greater than MIN_DISTANCE
     * @param targetPos Target position to got to
     * @param currentPosition Previous target positions
     * @return Boolean that tells whether the target position has been reached
     */
    static bool isTargetReached(const Vector2 &targetPos, const Vector2 &currentPosition);

    /**
     * @brief Removes the first element in the path in the case the first point is reached
     * @param path the vector of path points
     * @param currentPosition the current position of the robot
     */
    static void removeFirstIfReached(std::vector<Vector2> &path, const Vector2 &currentPosition);
};
}  // namespace rtt::ai::control

#endif  // RTT_POSITIONCONTROLUTILS_H

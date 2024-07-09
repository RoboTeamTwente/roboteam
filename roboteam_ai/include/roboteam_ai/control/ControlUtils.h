#ifndef ROBOTEAM_AI_CONTROLUTILS_H
#define ROBOTEAM_AI_CONTROLUTILS_H

#include <roboteam_utils/Arc.h>
#include <roboteam_utils/Grid.h>
#include <roboteam_utils/Line.h>
#include <utilities/StpInfoEnums.h>

#include <cmath>
#include <optional>
#include <roboteam_utils/Field.hpp>

#include "utilities/Constants.h"

using Vector2 = rtt::Vector2;

namespace rtt::ai::control {

/**
 * Class with useful functions used for robot control
 */
class ControlUtils {
   public:
    /**
     * @brief Determines the kick force based on the distance and the type of kick
     * @param distance distance to the target
     * @param shotPower type of the kick
     * @return a kick speed between min and max kick speed
     */
    static double determineKickForce(const double distance, stp::ShotPower shotPower) noexcept;
    /**
     * @brief Determines the chip force based on the distance and the type of chip
     * @param distance distance to the target
     * @return a chip speed between min and max chip speed
     */
    static double determineChipForce(const double distance) noexcept;

    /**
     * @brief Determine the max allowed velocity considering the game state and whether the robot has the ball
     * @param hasBall Whether this robot has the ball
     * @return The max allowed velocity for this robot
     */
    static double getMaxVelocity(bool hasBall);
};

}  // namespace rtt::ai::control

#endif  // ROBOTEAM_AI_CONTROLUTILS_H

//
// Created by baris on 16/11/18.
//

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
using Angle = rtt::Angle;

namespace rtt::ai::control {

/**
 * Class with useful functions used for robot control
 */
class ControlUtils {
   public:
    /**
     * @brief Calculates the force for a given vector and weight
     * @param vector Direction of the force
     * @param weight Weight that needs to be displaced
     * @param minDistance Minimum distance the weight should move
     * @return Force needed to displace the weight
     */
    static Vector2 calculateForce(const rtt::Vector2 &vector, double weight, double minDistance);

    /**
     * @brief Limits the velocity
     * @param vel current velocity
     * @param maxVel maximum velocity
     * @param minVel minimum velocity
     * @param listenToReferee listen to or ignore the referee
     * @return limited velocity
     */
    static Vector2 velocityLimiter(const Vector2 &vel, double maxVel = Constants::MAX_VEL(), double minVel = 0.0, bool listenToReferee = true);

    /**
     * @brief Limits the acceleration
     * @param targetVel velocity the robot should be at
     * @param prevVel velocity of the robot in the previous tick
     * @param targetAngle the angle the robot should be at
     * @param sidewaysAcceleration maximum sideways acceleration
     * @param forwardsAcceleration maximum forward acceleration
     * @param sidewaysDeceleration maximum sideways deceleration
     * @param forwardsDeceleration maximum forward deceleration
     * @return limited acceleration
     */
    static Vector2 accelerationLimiter(const Vector2 &targetVel, const Vector2 &prevVel, const Angle &targetAngle,
                                       double sidewaysAcceleration = Constants::MAX_ACC_LOWER() / Constants::STP_TICK_RATE(),
                                       double forwardsAcceleration = Constants::MAX_ACC_UPPER() / Constants::STP_TICK_RATE(),
                                       double sidewaysDeceleration = Constants::MAX_DEC_LOWER() / Constants::STP_TICK_RATE(),
                                       double forwardsDeceleration = Constants::MAX_DEC_UPPER() / Constants::STP_TICK_RATE());

    /**
     * @brief calculates whether the object velocity will collide with a point
     * @param objectPosition position of the object
     * @param velocity velocity vector
     * @param point point to check for collision
     * @param maxDifference maximum difference between vector and point
     * @return boolean that tells whether the object will collide with the point
     */
    static bool objectVelocityAimedToPoint(const Vector2 &objectPosition, const Vector2 &velocity, const Vector2 &point, double maxDifference = 0.3);

    /**
     * @brief Determines the kick force based on the distance and the type of kick
     * @param distance distance to the target
     * @param shotType type of the kick
     * @return a kick speed between min and max kick speed
     */
    static double determineKickForce(const double distance, stp::ShotType shotType) noexcept;
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

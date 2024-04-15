#ifndef RTT_OVERSHTOOTCOMPUTATIONS_H
#define RTT_OVERSHTOOTCOMPUTATIONS_H

#include <roboteam_utils/Vector2.h>

#include <cmath>
#include <optional>
#include <vector>

#include "BBTrajectories/BBTrajectory1D.h"

namespace rtt::ai::control {

/**
 * @brief Class that computes the overshooting destination of the robot when it can not reach the target position in time with a full stop.
 * Based on TIGERs TDP 2023 (https://tdp.roboteamtwente.nl/static/tdps/144/tdp.html). Implementation is based on their code release of 2023.
 * Note that variable names have been kept the same.
 */
struct TimedPos1D {
    double pos;
    double time;

    TimedPos1D(double pos, double time) : pos(pos), time(time) {}
};
class OvershootComputations {
   public:
    /**
     * @brief Calculate the overshooting destination of the robot.
     * 
     * @param startPosition The start position of the robot.
     * @param endPosition The target position of the robot.
     * @param startVelocity The start velocity of the robot.
     * @param maxVelocity The maximum velocity of the robot.
     * @param maxAcceleration The maximum acceleration of the robot.
     * @param targetTime The time it takes to reach the target position.
     * @return Vector2 The overshooting destination of the robot.
    */
    static Vector2 overshootingDestination(Vector2 &startPosition, Vector2 &endPosition, Vector2 &startVelocity, double maxVelocity, double maxAcceleration, double targetTime);

   private:
    /**
     * @brief Calculate the time it takes to reach the target position with the slowest possible direct movement.
     * 
     * @param s The distance to the target position.
     * @param v0 The initial velocity of the robot.
     * @param aMax The maximum acceleration of the robot.
     * @return double The time it takes to reach the target position.
     */
    static double calcSlowestDirectTime(double s, double v0, double aMax);

    /**
     * @brief Calculate the fastest possible direct movement with a trapezoidal form.
     * 
     * @param s The distance to the target position.
     * @param v0 The initial velocity of the robot.
     * @param v1Max The maximum velocity of the robot.
     * @param aMax The maximum acceleration of the robot.
     * @param aDec The maximum deceleration of the robot.
     * @param tt The time it takes to reach the target position.
     * @return std::optional<TimedPos1D> The fastest possible direct movement with a trapezoidal form.
    */
    static std::optional<TimedPos1D> calcFastestDirectTrapezoidal(double s, double v0, double v1Max, double aMax, double aDec, double tt);

    /**
     * @brief Calculate the fastest possible direct movement with a triangular form.
     * 
     * @param s The distance to the target position.
     * @param v0 The initial velocity of the robot.
     * @param v1Max The maximum velocity of the robot.
     * @param aMax The maximum acceleration of the robot.
     * @param aDec The maximum deceleration of the robot.
     * @param tt The time it takes to reach the target position.
     * @return TimedPos1D The fastest possible direct movement with a triangular form.
    */
    static TimedPos1D calcFastestDirectTriangular(double s, double v0, double v1Max, double aMax, double aDec, double tt);

    /**
     * @brief Calculate the fastest possible direct movement with a triangular form.
     * 
     * @param s The distance to the target position.
     * @param v0 The initial velocity of the robot.
     * @param v1Max The maximum velocity of the robot.
     * @param aMax The maximum acceleration of the robot.
     * @param aDec The maximum deceleration of the robot.
     * @param tt The time it takes to reach the target position.
    */
    static TimedPos1D getTimedPos1D(double s, double v0, double vMax, double aMax, double tt);
};
}  // namespace rtt::ai::control
#endif  // RTT_OVERSHTOOTCOMPUTATIONS_H

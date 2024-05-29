#ifndef RTT_OVERSHOOTCOMPUTATIONS_H
#define RTT_OVERSHOOTCOMPUTATIONS_H

#include <roboteam_utils/Vector2.h>

#include <cmath>
#include <optional>
#include <vector>

#include "BBTrajectories/BBTrajectory1D.h"

namespace rtt::ai::control {

/**
 * @brief Class that computes the overshooting destination of the robot when it can not reach the target position in time with a full stop.
 * Based on TIGERs TDP 2023 (https://tdp.roboteamtwente.nl/static/tdps/144/tdp.html). Implementation is based on their code release of 2023.
 */
struct TimedPos1D {
    double pos;
    double time;
    double timeToTarget;

    TimedPos1D(double pos, double time, double timeToTarget) : pos(pos), time(time), timeToTarget(timeToTarget) {}
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
    static std::pair<Vector2, double> overshootingDestination(const Vector2 &startPosition, const Vector2 &endPosition, const Vector2 &startVelocity, double maxVelocity,
                                                              double maxAcceleration, double targetTime);

   private:
    /**
     * @brief Calculate the time it takes to reach the target position with the slowest possible direct movement.
     *
     * @param distance The distance to the target position.
     * @param initialVelocity The initial velocity of the robot.
     * @param maxAcceleration The maximum acceleration of the robot.
     * @return double The time it takes to reach the target position.
     */
    static double slowestDirectTime(double distance, double initialVelocity, double maxAcceleration);

    /**
     * @brief Calculate the fastest possible direct movement with any form.
     *
     * @param distance The distance to the target position.
     * @param initialVelocity The initial velocity of the robot.
     * @param maxVelocity The maximum velocity of the robot.
     * @param maxAcceleration The maximum acceleration of the robot.
     * @param targetTime The time it takes to reach the target position.
     * @return The fastest possible direct movement with any form.
     */
    static TimedPos1D fastestDirect(double distance, double initialVelocity, double maxVelocity, double maxAcceleration, double targetTime);

    /**
     * @brief Calculate the fastest possible direct movement with a trapezoidal form.
     *
     * @param distance The distance to the target position.
     * @param initialVelocity The initial velocity of the robot.
     * @param maxVelocity The maximum velocity of the robot.
     * @param maxAcceleration The maximum acceleration of the robot.
     * @param deceleration The maximum deceleration of the robot.
     * @param targetTime The time it takes to reach the target position.
     * @return std::optional<TimedPos1D> The fastest possible direct movement with a trapezoidal form.
     */
    static std::optional<TimedPos1D> fastestDirectTrapezoidal(double distance, double initialVelocity, double maxVelocity, double maxAcceleration, double deceleration,
                                                              double targetTime);

    /**
     * @brief Calculate the fastest possible direct movement with a triangular form.
     *
     * @param distance The distance to the target position.
     * @param initialVelocity The initial velocity of the robot.
     * @param maxVelocity The maximum velocity of the robot.
     * @param maxAcceleration The maximum acceleration of the robot.
     * @param deceleration The maximum deceleration of the robot.
     * @param targetTime The time it takes to reach the target position.
     * @return TimedPos1D The fastest possible direct movement with a triangular form.
     */
    static TimedPos1D fastestDirectTriangular(double distance, double initialVelocity, double maxVelocity, double maxAcceleration, double deceleration, double targetTime);

    /**
     * @brief Calculate the fastest possible direct movement with a triangular form.
     *
     * @param distance The distance to the target position.
     * @param initialVelocity The initial velocity of the robot.
     * @param maxVelocity The maximum velocity of the robot.
     * @param maxAcceleration The maximum acceleration of the robot.
     * @param targetTime The time it takes to reach the target position.
     * @return TimedPos1D The fastest possible direct movement with a triangular form.
     */
    static TimedPos1D getTimedPos1D(double distance, double initialVelocity, double maxVelocity, double maxAcceleration, double targetTime);
};

}  // namespace rtt::ai::control

#endif  // RTT_OVERSHOOTCOMPUTATIONS_H
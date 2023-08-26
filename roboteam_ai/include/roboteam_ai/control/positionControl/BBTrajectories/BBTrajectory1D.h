//
// Created by rolf on 26-09-20.
//

#ifndef RTT_BBTRAJECTORY1D_H
#define RTT_BBTRAJECTORY1D_H

#include <array>
#include <vector>

namespace rtt::BB {
/**
 * @brief Represents a segment of a one-dimensional bang-bang trajectory.
 */
struct BBTrajectoryPart {
    double tEnd;     /**< Time it takes to get to the end of trajectory part */
    double acc;      /**< Acceleration of the current trajectory part */
    double startVel; /**< Velocity at the start of the trajectory part */
    double startPos; /**< Position at the start of the trajectory part */
};

/**
 * @brief small struct to keep Position, Velocity and Acceleration in at once.
 */
struct BBPosVel {
    /**
     * @brief Explicit constructor for th BBPosVelAcc struct
     * @param pos Position
     * @param vel Velocity
     *
     */
    explicit BBPosVel(double pos, double vel) : pos{pos}, vel{vel} {};

    double pos; /**< Position */
    double vel; /**< Velocity */
};

/**
 * @author Rolf
 * @brief Class that represents 1 dimensional bang bang trajectories.
 * These are time optimal in one dimension given that bang-bang control where you pick an acceleration in (max,0,-max) is possible.
 * In this case the trajectory will have 1-3 segment.
 */

class BBTrajectory1D {
   public:
    /**
     * @brief Gets the position and velocity at time t
     * @param t time to get values at
     * @return The PosVel
     */
    [[nodiscard]] BBPosVel getValues(double t) const;

    /**
     * @brief Gets the position at time t
     * @param t time to get position at
     * @return Position at time t
     */
    [[nodiscard]] double getPosition(double t) const;

    /**
     * @brief Gets the velocity at time t
     * @param t time to get velocity at
     * @return Velocity at time t
     */
    [[nodiscard]] double getVelocity(double t) const;

    /**
     * @brief Gets the acceleration at time t
     * @param t time to get acceleration at.
     * @return Acceleration at time t
     */
    [[nodiscard]] double getAcceleration(double t) const;

    /**
     * @brief Gets the total trajectory time.
     * @return Total time of the trajectory to end point
     */
    [[nodiscard]] double getTotalTime() const;

    /**
     * @brief Computes the position where we would end up if we use maximum deceleration
     * @param pos Current position
     * @param vel Current velocity
     * @param maxAcc Maximum allowed acceleration/deceleration
     * @return final position (with zero velocity)
     */
    static double fullBrakePos(double pos, double vel, double accMax);

    /**
     * Checks if time is in the last (decelerating to 0) part of the trajectory
     * @param t
     * @return true if t is in the last part of the trajectory
     */
    [[nodiscard]] bool inLastPart(double t) const;

    /**
     * Generate a time-optimal trajectory given the parameters and a bang-bang control model where we
     * either accelerate/decelerate at max deceleration or drive at the max velocity.
     * @param initialPos Current position
     * @param initialVel Current velocity
     * @param finalPos Final position
     * @param maxVel Maximum allowed velocity (absolute)
     * @param maxAcc maximum allowed acceleration/deceleration
     */
    void generateTrajectory(double initialPos, double initialVel, double finalPos, double maxVel, double maxAcc);

    /**
     * @brief Puts all BBT parts in a vector
     * @return All BBT parts as a vector
     */
    std::vector<BB::BBTrajectoryPart> getParts();

    /**
     * @brief Constructor for the BBTrajectory1D class
     * @param initialPos Initial position of the robot
     * @param initialVel Initial velocity of the robot
     * @param finalPos Target position to go to
     * @param maxVel Maximum allowed velocity
     * @param maxAcc Maximum allowed acceleration or deceleration
     */
    BBTrajectory1D(double initialPos, double initialVel, double finalPos, double maxVel, double maxAcc);

    /**
     * @brief Default constructor for the BBTrajectory1D class
     */
    BBTrajectory1D() = default;

   private:
    /**
     * Computes the position where we would end if we first accelerate to a target velocity and then immediately decelerate to 0
     * @param pos0 Current position
     * @param vel0 Current velocity
     * @param vel1 Target velocity
     * @param accMax Maximum allowed acceleration/deceleration
     * @return final position (with zero velocity)
     */
    static double accelerateBrakePos(double pos0, double vel0, double vel1, double accMax);

    /**
     * Generates a time-optimal triangular profile (accelerating and then breaking)
     * @param initialPos Current position
     * @param initialVel Current velocity
     * @param finalPos Desired final psition
     * @param maxAcc Maximum allowed acceleration/deceleration
     * @param invertedSign Specifies whether or not to invert the sign of velocity / acceleration. This is more of a techniality
     */
    void triangularProfile(double initialPos, double initialVel, double finalPos, double maxAcc, bool invertedSign);

    /**
     * Generates a time-optimal trapezoidal profile (accelerating, coasting, then breaking)
     * @param initialPos Current position
     * @param initialVel Current velocity
     * @param maxVel Coasting velocity
     * @param finalPos Desired final position
     * @param maxAcc Maximum allowed acceleration/deceleration
     */
    void trapezoidalProfile(double initialPos, double initialVel, double finalPos, double maxVel, double maxAcc);

    /**
     * Updates a part of the trajectory
     * @param index which part to update
     * @param tEnd time spent on trajectory segment
     * @param acc acceleration on segment
     * @param vel initial velocity of segment
     * @param pos initial position of segment
     */
    void updatePart(int index, double tEnd, double acc, double vel, double pos);

    std::array<BBTrajectoryPart, 3> parts; /**< Array of length 3 containing all parts of a BBT */
    unsigned short int numParts = 0;       /**< Unsigned short integer which stores the amount of parts used in the BBT */
    double finalPos;                       /**< Target position to go to */
    double maxVel;                         /**< Maximum allowed velocity */
};
}  // namespace rtt::BB

#endif  // RTT_BBTRAJECTORY1D_H

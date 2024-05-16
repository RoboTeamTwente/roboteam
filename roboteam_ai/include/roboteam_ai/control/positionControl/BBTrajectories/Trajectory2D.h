#ifndef RTT_TRAJECTORY2D_H
#define RTT_TRAJECTORY2D_H

#include <vector>

#include "Trajectory1D.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {

/**
 * @author Tijmen
 * @brief Class that stores 2 dimensional trajectories
 */
class Trajectory2D {
   public:
    /**
     * @brief Default constructor of the Trajectory2D class
     */
    Trajectory2D() = default;

    /**
     * @brief Constructor of the Trajectory2D class
     * @param initialPos Initial position of the robot
     * @param initialVel Initial velocity of the robot
     * @param finalPos Target position to got to
     * @param maxVel Maximum allowed velocity
     * @param maxAcc Maximum allowed acceleration or deceleration
     */
    Trajectory2D(const Vector2 &initialPos, const Vector2 &initialVel, const Vector2 &finalPos, double maxVel, double maxAcc);

    /**
     * @brief Stores the trajectory
     * @param newParts Trajectory which needs to be stored
     * @param addFromTime Time the trajectory was created
     */
    void addTrajectory(const Trajectory2D &extraTrajectory, double addFromTime);

    /**
     * @brief Approaches the Trajectory by dividing the path in points which are separated by timeStep seconds
     * @param timeStep time between pathpoints
     * @return Vector of directions which make up the trajectory
     */
    [[nodiscard]] std::vector<Vector2> getPathApproach(double timeStep) const;

    /**
     * @brief Retrieves the vector which stores the velocities of each trajectory part
     * @param timeStep time between pathpoints
     * @return Vector of velocities which follow the trajectory
     */
    [[nodiscard]] std::vector<Vector2> getVelocityVector(double timeStep) const;

    /**
     * @brief Get the position in the trajectory at time t
     * @param t Given time.
     * @return Position at time t
     */
    [[nodiscard]] Vector2 getPosition(double t) const;

    /**
     * @brief Get the velocity in the trajectory at time t
     * @param t Given time.
     * @return Velocity at time t
     */
    [[nodiscard]] Vector2 getVelocity(double t) const;

    /**
     * @brief Get the acceleration in the trajectory at time t
     * @param t Given time.
     * @return Acceleration at time t
     */
    [[nodiscard]] Vector2 getAcceleration(double t) const;

    /**
     * @brief Gets the total trajectory time.
     * @return Total time of the trajectory to end point
     */
    [[nodiscard]] double getTotalTime() const;

   private:
    Trajectory1D x; /**< 1D x component of the 2D Trajectory */
    Trajectory1D y; /**< 1D y component of the 2D Trajectory */
};

}  // namespace rtt

#endif  // RTT_TRAJECTORY2D_H

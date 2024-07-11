#ifndef RTT_TRAJECTORY2D_H
#define RTT_TRAJECTORY2D_H

#include <ruckig/ruckig.hpp>
#include <vector>

#include "Trajectory1D.h"
#include "roboteam_utils/Vector2.h"

using namespace ruckig;
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
     * @param initialAcc Initial acceleration of the robot
     * @param finalPos Target position to got to
     * @param maxVel Maximum allowed velocity
     * @param maxAcc Maximum allowed acceleration or deceleration
     * @param maxJerk Maximum allowed jerk
     */
    Trajectory2D(const Vector2 &initialPos, const Vector2 &initialVel, const Vector2 &initialAcc, const Vector2 &finalPos, double maxVel, double maxAcc, double maxJerk);

    /**
     * @brief Constructor of the Trajectory2D class, from a robot ID instead of initial acceleration
     * @param initialPos Initial position of the robot
     * @param initialVel Initial velocity of the robot
     * @param finalPos Target position to got to
     * @param maxVel Maximum allowed velocity
     * @param maxAcc Maximum allowed acceleration or deceleration
     * @param maxJerk Maximum allowed jerk
     */
    Trajectory2D(const Vector2 &initialPos, const Vector2 &initialVel, const Vector2 &finalPos, double maxVel, double maxAcc, double maxJerk, int robotId);

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
    static std::unordered_map<int, Vector2> lastAcceleration; /**< Map of last acceleration for each robot */

   private:
    Trajectory<1> x; /**< 1D x component of the 2D Trajectory */
    std::optional<Trajectory<1>> xAdded;
    Trajectory<1> y; /**< 1D y component of the 2D Trajectory */
    std::optional<Trajectory<1>> yAdded;
    double addedFromTime = std::numeric_limits<double>::infinity();
};

}  // namespace rtt

#endif  // RTT_TRAJECTORY2D_H

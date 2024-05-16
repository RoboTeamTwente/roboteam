#ifndef RTT_TRAJECTORY1D_H
#define RTT_TRAJECTORY1D_H

#include "BBTrajectory1D.h"

namespace rtt {
/**
 * @author Tijmen
 * @brief Class that stores 1 dimensional trajectories
 */
class Trajectory1D {
   public:
    /**
     * @brief Stores the trajectory
     * @param newParts Trajectory which needs to be stored
     * @param addFromTime Time the trajectory was created
     */
    void addTrajectory(const std::vector<rtt::ai::control::BBTrajectoryPart> &newParts, double addFromTime);

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

    std::vector<rtt::ai::control::BBTrajectoryPart> parts; /**< Vector containing all parts of the trajectory */
    double finalPos;                                       /**< Target position to go to */
    double maxVel;                                         /**< Maximum allowed velocity */
};

}  // namespace rtt

#endif  // RTT_TRAJECTORY1D_H

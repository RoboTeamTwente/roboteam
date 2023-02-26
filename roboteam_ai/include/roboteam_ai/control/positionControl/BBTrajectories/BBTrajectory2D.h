//
// Created by rolf on 26-09-20.
//

#ifndef RTT_BBTRAJECTORY2D_H
#define RTT_BBTRAJECTORY2D_H

#include <roboteam_utils/Vector2.h>

#include <vector>

#include "BBTrajectory1D.h"
#include "control/positionControl/StateVector.h"

namespace rtt::ai::control {

/**
 * @author Rolf
 * @brief Class that computes and stores 2 dimensional bang-bang trajectories. These can compute close to time-optimal
 * trajectories in 2 dimensions.
 */
class BBTrajectory2D {
   public:
    class Iterator  {
       public:
        using T = std::pair<double, StateVector>;
        using difference_type = std::ptrdiff_t;
        using value_type = T;
        using pointer = T*;
        using reference = T&;
        using iterator_category = std::output_iterator_tag;

        explicit Iterator(const BBTrajectory2D& trajectory, double time = 0);

        reference operator*();
        pointer operator->();
        Iterator& operator++();
        bool operator==(const Iterator& other) const;
        bool operator!=(const Iterator& other) const;

       private:
        const BBTrajectory2D& trajectory;
        std::optional<T> item;
        double time;
    };

    /**
     * @brief Default constructor
     */
    BBTrajectory2D() = default;

    /**
     * @brief Computes a roughly time-optimal bang-bang trajectory. This means that the velocity at the final point will
     * be 0. Any points on the trajectory will always satisfy the maxVel and maxAcc constraints.
     * @param initialPos The initial position to start the trajectory from
     * @param initialVel The initial velocity to start the trajectory from
     * @param finalPos The final position to arrive at.
     * @param maxVel The maximum allowed velocity for this path.
     * @param maxAcc The maximum allowed acceleration allowed for the robot on this path
     */
    BBTrajectory2D(const Vector2 &initialPos, const Vector2 &initialVel, const Vector2 &finalPos, double maxVel, double maxAcc);

    /**
     * @brief  Computes a time-optimal bang-bang trajectory.
     * Uses binary search to search over some trajectories to find one where both the x and y dimension take roughly the same amount
     * of time.
     * @param initialPos The initial position to start the trajectory from
     * @param initialVel The initial velocity to start the trajectory from
     * @param finalPos The final position to arrive at.
     * @param maxVel The maximum allowed velocity for this path.
     * @param maxAcc The maximum allowed acceleration allowed for the robot on this path
     */
    void generateSyncedTrajectory(const Vector2 &initialPos, const Vector2 &initialVel, const Vector2 &finalPos, double maxVel, double maxAcc);

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
     * @brief Gets tEnd of the current part
     */
    [[maybe_unused]] [[nodiscard]] double getTotalTime() const;

    /**
     * @brief Returns all the trajectory parts in both dimensions to use in the general trajectory class
     */
    [[nodiscard]] std::pair<std::vector<BBTrajectoryPart>, std::vector<BBTrajectoryPart>> getParts();

    Iterator begin() const { return Iterator(*this, 0); }
    Iterator end() const { return Iterator(*this, getTotalTime()); }

   private:
    /**
     * @brief  Computes a bang bang trajectory with a given alpha value.
     * This is NOT time optimal and may give very 'unphysical' paths for high or low alpha value.
     * Don't use this function unless you know what you are doing.
     * @param initialPos The initial position to start the trajectory from
     * @param initialVel The initial velocity to start the trajectory from
     * @param finalPos The final position to arrive at.
     * @param maxVel The maximum allowed velocity for this path.
     * @param maxAcc The maximum allowed acceleration allowed for the robot on this path
     * @param alpha The chosen angle, should be between 0 and M_PI_2. Angle 0 gives all of the control to the x dimension
     * whilst at M_PI all of the velocity/acceleration is given to y dimension.
     */
    void generateTrajectory(const Vector2 &initialPos, const Vector2 &initialVel, const Vector2 &finalPos, double maxVel, double maxAcc, double alpha);

    BBTrajectory1D x;
    BBTrajectory1D y;
};
}  // namespace rtt::BB
#endif  // RTT_BBTRAJECTORY2D_H

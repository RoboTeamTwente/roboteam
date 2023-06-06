//
// Created by ratoone on 20-02-20.
//

#ifndef RTT_NUMTREESPLANNING_H
#define RTT_NUMTREESPLANNING_H

#include <queue>

#include "PathPlanningAlgorithm.h"
#include "control/positionControl/CollisionDetector.h"
#include "control/positionControl/PathPointNode.h"
#include "utilities/Constants.h"

namespace rtt::ai::control {

class NumTreesPlanning : public PathPlanningAlgorithm {
   private:
    static constexpr double AVOIDANCE_DISTANCE = 5 * Constants::ROBOT_RADIUS(); /**< Minimum distance the robot should keep when avoiding */
    static constexpr double TARGET_THRESHOLD = 0.1; /**< Maximum distance the robot can be from the target position for it to be successful */
    static constexpr int MAX_BRANCHING = 10; /**< Maximum amount of different trajectories that can be taken into account */
    static constexpr int MAX_ITERATIONS = 10; /**< Maximum amount of times the trajectories can be evaluated */

    /**
     * Generate 2 new points to the side of the collisionPosition, such that the points and the parent point form
     * an isosceles triangle, with the collisionPosition being the middle of the base. The first point will be the one
     * closest to the destination.
     * @param parentPoint starting point of the robot
     * @param collisionPosition the point to branch from
     * @param destination the target position.
     * @return Vector containing path points of the branched path
     */
    std::vector<PathPointNode> branchPath(PathPointNode &parentPoint, const Vector2 &collisionPosition, Vector2 &destination) const;

   public:
    /**
     * @brief Explicit constructor of the NumTreesPlanning class
     * @param collisionDetector Detects collisions allong the path
     */
    explicit NumTreesPlanning(CollisionDetector &collisionDetector);

    ~NumTreesPlanning() override = default;

    /**
     * Computes a path using the implemented algorithm. It takes into account the
     * obstacles present in the field. <br><br>
     * NumTreesPlanning uses an algorithm written by an old team member. It tries to
     * trace a path to the destination, and if there is a collision, it traces back
     * and branches into two points to the side of the obstacle, trying them afterwards.
     * @param robotPosition the current robot position
     * @param targetPosition the goal position
     * @return a vector of points representing the path
     */
    std::vector<Vector2> computePath(const Vector2 &robotPosition, Vector2 &targetPosition) override;
};
}  // namespace rtt::ai::control

#endif  // RTT_NUMTREESPLANNING_H

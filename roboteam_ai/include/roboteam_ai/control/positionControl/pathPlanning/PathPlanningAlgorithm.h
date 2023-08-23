//
// Created by ratoone on 09-03-20.
//

#ifndef RTT_PATHPLANNINGALGORITHM_H
#define RTT_PATHPLANNINGALGORITHM_H

#include "control/positionControl/CollisionDetector.h"

namespace rtt::ai::control {
/**
 * @brief The base class for the path planning algorithms. All future algorithms should inherit from this
 */
class PathPlanningAlgorithm {
   protected:
    CollisionDetector &collisionDetector; /**< Detects collisions on the trajectory */

   public:
    /**
     * Constructor of the PathPlanningAlgorithm class
     * @param collisionDetector Detects collisions on the trajectory
     */
    explicit PathPlanningAlgorithm(CollisionDetector &collisionDetector);

    /**
     * @brief Virtual destructor of the PathPlanningAlgorithm
     */
    virtual ~PathPlanningAlgorithm() = default;

    /**
     * @brief Algorithm specific path computation. It should take into account the obstacles in the field
     * @param robotPosition the current robot position
     * @param targetPosition the target position to move to
     * @return a vector of points representing the path
     */
    virtual std::vector<Vector2> computePath(const Vector2 &robotPosition, Vector2 &targetPosition) = 0;
};
}  // namespace rtt::ai::control

#endif  // RTT_PATHPLANNINGALGORITHM_H

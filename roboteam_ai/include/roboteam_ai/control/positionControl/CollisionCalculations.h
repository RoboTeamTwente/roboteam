#ifndef RTT_COLLISIONCALCULATIONS_H
#define RTT_COLLISIONCALCULATIONS_H

#include <roboteam_utils/RobotCommands.hpp>

#include "control/positionControl/BBTrajectories/Trajectory2D.h"
#include "utilities/GameStateManager.hpp"
#include "utilities/StpInfoEnums.h"
#include "world/FieldComputations.h"

namespace rtt::ai::control {

/**
 * @brief This class is responsible for performing collision calculations.
 */
class CollisionCalculations {
    public:
     /**
      * @brief Checks if a trajectory collides with a motionless object.
      * @param Trajectory The trajectory to check.
      * @param avoidObjects The objects to avoid.
      * @param field The field on which the trajectory is.
      * @param completedTimeSteps The number of completed time steps.
      * @return True if the trajectory collides with a motionless object, false otherwise.
      */
     static bool isCollidingWithMotionlessObject(const Trajectory2D &Trajectory, stp::AvoidObjects avoidObjects, const Field &field, int &completedTimeSteps);

     /**
      * @brief Gets the first collision time with a motionless object.
      * @param Trajectory The trajectory to check.
      * @param avoidObjects The objects to avoid.
      * @param field The field on which the trajectory is.
      * @param completedTimeSteps The number of completed time steps.
      * @return The first collision time with a motionless object.
      */
     static double getFirstCollisionTimeMotionlessObject(const Trajectory2D &Trajectory, stp::AvoidObjects avoidObjects, const Field &field, int &completedTimeSteps);

     /**
      * @brief Checks if a trajectory collides with a moving object.
      * @param Trajectory The trajectory to check.
      * @param avoidObjects The objects to avoid.
      * @param robotId The ID of the robot.
      * @param completedTimeSteps The number of completed time steps.
      * @param world The world in which the trajectory and objects are.
      * @param computedPaths The computed paths of the robots.
      * @return True if the trajectory collides with a moving object, false otherwise.
      */
     static bool isCollidingWithMovingObject(const Trajectory2D &Trajectory, stp::AvoidObjects avoidObjects, int &robotId, int &completedTimeSteps,
                                                          const world::World *world, const std::unordered_map<int, std::vector<Vector2>> &computedPaths);

     /**
      * @brief Gets the first collision time with a moving object.
      * @param Trajectory The trajectory to check.
      * @param avoidObjects The objects to avoid.
      * @param robotId The ID of the robot.
      * @param completedTimeSteps The number of completed time steps.
      * @param world The world in which the trajectory and objects are.
      * @param computedPaths The computed paths of the robots.
      * @return The first collision time with a moving object.
      */
     static double getFirstCollisionTimeMovingObject(const Trajectory2D &Trajectory, stp::AvoidObjects avoidObjects, int &robotId, int &completedTimeSteps,
                                                                     const world::World *world, const std::unordered_map<int, std::vector<Vector2>> &computedPaths);

     /**
      * @brief Gets the first collision time with any object.
      * @param Trajectory The trajectory to check.
      * @param avoidObjects The objects to avoid.
      * @param field The field on which the trajectory is.
      * @param robotId The ID of the robot.
      * @param completedTimeSteps The number of completed time steps.
      * @param world The world in which the trajectory and objects are.
      * @param computedPaths The computed paths of the robots.
      * @return The first collision time with any object.
      */
     static double getFirstCollisionTime(const Trajectory2D &Trajectory, stp::AvoidObjects avoidObjects, const Field &field, int &robotId, int &completedTimeSteps,
                                                     const world::World *world, const std::unordered_map<int, std::vector<Vector2>> &computedPaths);

     /**
      * @brief Checks if a trajectory collides with any object.
      * @param Trajectory The trajectory to check.
      * @param avoidObjects The objects to avoid.
      * @param field The field on which the trajectory is.
      * @param robotId The ID of the robot.
      * @param completedTimeSteps The number of completed time steps.
      * @param world The world in which the trajectory and objects are.
      * @param computedPaths The computed paths of the robots.
      * @return True if the trajectory collides with any object, false otherwise.
      */
     static bool isColliding(const Trajectory2D &Trajectory, stp::AvoidObjects avoidObjects, const Field &field, int &robotId, int &completedTimeSteps, const world::World *world,
                                     const std::unordered_map<int, std::vector<Vector2>> &computedPaths);
};
}  // namespace rtt::ai::control

#endif  // RTT_COLLISIONCALCULATIONS_H
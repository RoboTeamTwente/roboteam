//
// Created by ratoone on 18-11-19.
//

#ifndef RTT_POSITIONCONTROL_H
#define RTT_POSITIONCONTROL_H

#include <roboteam_utils/RobotCommands.hpp>

#include "BBTrajectories/Trajectory2D.h"
#include "CollisionDetector.h"
#include "control/positionControl/BBTrajectories/WorldObjects.h"
#include "control/positionControl/pathPlanning/NumTreesPlanning.h"
#include "control/positionControl/pathTracking/BBTPathTracking.h"
#include "control/positionControl/pathTracking/DensePathTracking.h"
#include "control/positionControl/pathTracking/PidTracking.h"
#include "utilities/StpInfoEnums.h"

namespace rtt::ai::control {

/**
 * @brief The main position control class. Use this for your robot position control requirements.
 */
class PositionControl {
   private:
    static constexpr double FINAL_AVOIDANCE_DISTANCE = 4 * Constants::ROBOT_RADIUS(); /**< Minimum distance the robot will keep to avoid collisions */
    CollisionDetector collisionDetector;                                              /**< Detects collisions on the trajectory */
    rtt::BB::WorldObjects worldObjects;                                               /**< Calculates collisions */
    NumTreesPlanning pathPlanningAlgorithm = NumTreesPlanning(collisionDetector);     /**< Creates new trajectories when a collision is detected */
    DensePathTracking pathTrackingAlgorithm;                                          /**< Tracks the path */
    BBTPathTracking pathTrackingAlgorithmBBT;                                         /**< Tracks the BBT path */

    std::unordered_map<int, Trajectory2D> computedTrajectories;                            /**< Map of computed trajectories for each robot */
    std::unordered_map<int, std::vector<Vector2>> computedPaths;                           /**< Map of computed paths for each robot */
    std::unordered_map<int, std::vector<Vector2>> computedPathsVel;                        /**< Map of computed velocities for each robot */
    std::unordered_map<int, std::vector<std::pair<Vector2, Vector2>>> computedPathsPosVel; /**< Map of pairs containing position and velocity for each robot */
    std::unordered_map<int, int> completedTimeSteps;                                       /**< Map of completed time steps for each robot */

    /**
     * @brief Checks if the current path should be recalculated:
     * @param world a pointer to the current world
     * @param field the field object, used onwards by the collision detector
     * @param robotId the ID of the robot for which the path is calculated
     * @param currentPosition the current position of the aforementioned robot
     * @param currentVelocity its velocity
     * @param targetPosition the desired position that the robot has to reachsho
     * @return Boolean that is 1 if the path needs to be recalculated
     */
    bool shouldRecalculateTrajectory(const rtt::world::World *world, const rtt::Field &field, int robotId, Vector2 targetPosition, const Vector2 &currentPosition,
                                     ai::stp::AvoidObjects);

   public:
    /**
     * @brief Retrieves the computed path of the given robot
     * @param ID ID of the robot who's path we want to retrieve
     * @return Path that of the given robot
     */
    std::vector<Vector2> getComputedPath(int ID);

    /**
     * @brief Generates a path according to the selected planning algorithm,
     * and tracks it using the selected tracking algorithm. In the case a collision
     * is detected (using the collision detector), the path is recalculated.
     * @param field the field object, used onwards by the collision detector
     * @param robotId the ID of the robot for which the path is calculated
     * @param currentPosition the current position of the aforementioned robot
     * @param currentVelocity its velocity
     * @param targetPosition the desired position that the robot has to reach
     * @param pidType The desired PID type (intercept, regular, keeper etc.)
     * @return a RobotCommand, which can be fed directly in the output
     */
    RobotCommand computeAndTrackPath(const rtt::Field &field, int robotId, const Vector2 &currentPosition, const Vector2 &currentVelocity, Vector2 &targetPosition,
                                     stp::PIDType pidType);

    /**
     * @brief Updates the robot view vector
     * @param robotPositions the position vector of the robots
     */
    void setRobotPositions(std::vector<Vector2> &robotPositions);

    /**
     * @brief The computed path should be recalculated if: <br>
     * - it is empty (no path yet) <br>
     * - the target position changed with at least MAX_TARGET_DEVIATION <br>
     * - the robot will collide with another one by the next path point (ignored if the robot is not moving)
     * @param targetPos final target position
     * @param robotId the ID of the current robot
     * @return true if one of the above conditions are true, false otherwise
     */
    bool shouldRecalculatePath(const Vector2 &currentPosition, const Vector2 &targetPos, const Vector2 &currentVelocity, int robotId);

    /**
     * @brief Generates a collision-free trajectory and tracks it. Returns also possibly
     * the location of a collision on the path if no correct path can be found
     * @param world a pointer to the current world
     * @param field the field object, used onwards by the collision detector
     * @param robotId the ID of the robot for which the path is calculated
     * @param currentPosition the current position of the aforementioned robot
     * @param currentVelocity its velocity
     * @param targetPosition the desired position that the robot has to reach
     * @param maxRobotVelocity the maximum velocity that the robot is allowed to have
     * @param pidType The desired PID type (intercept, regular, keeper etc.)
     * @return A RobotCommand and optional with the location of the first collision on the path
     */
    rtt::BB::CommandCollision computeAndTrackTrajectory(const rtt::world::World *world, const rtt::Field &field, int robotId, Vector2 currentPosition, Vector2 currentVelocity,
                                                        Vector2 targetPosition, double maxRobotVelocity, stp::PIDType pidType, stp::AvoidObjects avoidObjects);

    /**
     * @brief Calculates a score for a trajectory. The score is based on the distance to the target and the
     * distance to the first collision on the path
     * @param world a pointer to the current world
     * @param field the field object, used onwards by the collision detector
     * @param firstCollision location of the first collision on the current path
     * @param trajectoryAroundCollision the trajectory to the intermediate point
     * @param avoidObjects whether or not to avoid objects
     * @param startTime the time at which the trajectory starts
     * @return A score for the trajectory
     */
    double calculateScore(const rtt::world::World *world, const rtt::Field &field, std::optional<BB::CollisionData> &firstCollision,
                          Trajectory2D &trajectoryAroundCollision, stp::AvoidObjects avoidObjects, double startTime = 0);

    /**
     * @brief Tries to find a new trajectory when the current path has a collision on it. It tries this by
     * looking for trajectories which go to intermediate points in the area of the collision and from these
     * paths again to the target
     * @param world the world object
     * @param field the field object, used onwards by the collision detector
     * @param robotId the ID of the robot for which the path is calculated
     * @param currentPosition the current position of the aforementioned robot
     * @param currentVelocity its velocity
     * @param firstCollision location of the first collision on the current path
     * @param targetPosition the desired position that the robot has to reach
     * @param maxRobotVelocity the maximum velocity the robot is allowed to have
     * @param timeStep the time between path points when approaching the path
     * @return An optional with a new path
     */
    std::optional<Trajectory2D> findNewTrajectory(const rtt::world::World *world, const rtt::Field &field, int robotId, Vector2 &currentPosition, Vector2 &currentVelocity,
                                                  std::optional<BB::CollisionData> &firstCollision, Vector2 &targetPosition, double maxRobotVelocity, double timeStep,
                                                  stp::AvoidObjects AvoidObjects);

    /**
     * @brief Creates intermediate points to make a path to. These points all have equal distance to the
     * collision point
     * @param field the field object, used onwards by the collision detector
     * @param firstCollision location of the first collision on the current path
     * @param targetPosition the desired position that the robot has to reach
     * @return A vector with coordinates of the intermediate points
     */
    std::vector<Vector2> createIntermediatePoints(const rtt::Field &field, std::optional<BB::CollisionData> &firstCollision, Vector2 &targetPosition);
};

}  // namespace rtt::ai::control
#endif  // RTT_POSITIONCONTROL_H

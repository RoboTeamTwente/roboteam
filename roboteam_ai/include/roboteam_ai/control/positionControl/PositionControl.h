#ifndef RTT_POSITIONCONTROL_H
#define RTT_POSITIONCONTROL_H

#include <roboteam_utils/RobotCommands.hpp>

#include "BBTrajectories/Trajectory2D.h"
#include "CollisionDetector.h"
#include "control/positionControl/pathTracking/BBTPathTracking.h"
#include "utilities/GameStateManager.hpp"
#include "utilities/StpInfoEnums.h"
#include "world/FieldComputations.h"

namespace rtt::ai::control {

/**
 * @brief The main position control class. Use this for your robot position control requirements.
 */
class PositionControl {
   private:
    static constexpr double MIN_ANGLE = 20 / M_PI;  /**< The minimum angle between the target and the intermediate point */
    static constexpr double MAX_ANGLE = 140 / M_PI; /**< The maximum angle between the target and the intermediate point */
    static constexpr double MIN_SCALE = 0.5;        /**< The minimum scale of the intermediate point with respect to start target distance */
    static constexpr double MAX_SCALE = 1.5;        /**< The maximum scale of the intermediate point with respect to start target distance */
    static constexpr int NUM_SUB_DESTINATIONS = 5;  /**< The number of sub destinations to create */
    CollisionDetector collisionDetector;            /**< Detects collisions on the trajectory */
    BBTPathTracking pathTrackingAlgorithmBBT;       /**< Tracks the BBT path */

    std::unordered_map<int, Trajectory2D> computedTrajectories;                            /**< Map of computed trajectories for each robot */
    std::unordered_map<int, std::vector<Vector2>> computedPaths;                           /**< Map of computed paths for each robot */
    std::unordered_map<int, std::vector<Vector2>> computedPathsVel;                        /**< Map of computed velocities for each robot */
    std::unordered_map<int, std::vector<std::pair<Vector2, Vector2>>> computedPathsPosVel; /**< Map of pairs containing position and velocity for each robot */
    std::unordered_map<int, int> completedTimeSteps;                                       /**< Map of completed time steps for each robot */
    std::unordered_map<int, Vector2> lastUsedNormalizedPoints;                             /**< Map of last used normalized points for each robot */

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
    RobotCommand computeAndTrackTrajectory(const rtt::world::World *world, const rtt::Field &field, int robotId, Vector2 currentPosition, Vector2 currentVelocity,
                                           Vector2 targetPosition, double maxRobotVelocity, stp::PIDType pidType, stp::AvoidObjects avoidObjects);

    /**
     * @brief Handles the collision with the ball at the current position. This function will calculate a new target, moving away from the ball as quickly as possible.
     * @param world a pointer to the current world
     * @param field the field object, used onwards by the collision detector
     * @param currentPosition the current position of the robot
     * @param avoidObjects whether or not to avoid objects
     * @return A new target position
     */
    Vector2 handleBallCollision(const rtt::world::World *world, const rtt::Field &field, Vector2 currentPosition, stp::AvoidObjects avoidObjects);

    /**
     * @brief Handles the collision with the ball placement at the current position, will move away from the ball placement location to ball line as quickly as possible.
     * @param world a pointer to the current world
     * @param field the field object, used onwards by the collision detector
     * @param currentPosition the current position of the robot
     * @param avoidObjects whether or not to avoid objects
     * @return A new target position
     */
    Vector2 handleBallPlacementCollision(const rtt::world::World *world, const rtt::Field &field, Vector2 currentPosition, stp::AvoidObjects avoidObjects);

    /**
     * @brief Handles the collision with the defense area at the current position, will move away from the defense area as quickly as possible.
     * @param field the field object, used onwards by the collision detector
     * @param currentPosition the current position of the robot
     * @return A new target position
     */
    Vector2 handleDefenseAreaCollision(const rtt::Field &field, Vector2 currentPosition);

    /**
     * @brief Tries to find a new trajectory when the current path has a collision on it. It tries this by
     * looking for trajectories which go to intermediate points in the area of the collision and from these
     * paths again to the target
     * @param world the world object
     * @param field the field object, used onwards by the collision detector
     * @param robotId the ID of the robot for which the path is calculated
     * @param currentPosition the current position of the aforementioned robot
     * @param currentVelocity its velocity
     * @param targetPosition the desired position that the robot has to reach
     * @param maxRobotVelocity the maximum velocity the robot is allowed to have
     * @param timeStep the time between path points when approaching the path
     * @return An optional with a new path
     */
    Trajectory2D findNewTrajectory(const rtt::world::World *world, const rtt::Field &field, int robotId, Vector2 &currentPosition, Vector2 &currentVelocity,
                                   Vector2 &targetPosition, double maxRobotVelocity, double timeStep, stp::AvoidObjects AvoidObjects);

    /**
     * @brief Creates normalized random points, which will be used to create intermediate points
     * @param robotId the ID of the robot for which the path is calculated
     * @return A vector with Vector2 of the normalized random points
     */
    std::vector<Vector2> generateNormalizedPoints(int robotId);
};

}  // namespace rtt::ai::control
#endif  // RTT_POSITIONCONTROL_H

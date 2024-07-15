#ifndef RTT_POSITIONCONTROL_H
#define RTT_POSITIONCONTROL_H

#include "BBTrajectories/Trajectory2D.h"
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

    std::unordered_map<int, Trajectory2D> computedTrajectories;  /**< Map of computed trajectories for each robot */
    std::unordered_map<int, std::vector<Vector2>> computedPaths; /**< Map of computed paths for each robot */
    std::unordered_map<int, Vector2> lastUsedNormalizedPoints;   /**< Map of last used normalized points for each robot */

   public:
    /**
     * @brief Retrieves the computed path of the given robot
     * @param ID ID of the robot who's path we want to retrieve
     * @return Path that of the given robot
     */
    std::vector<Vector2> getComputedPath(int ID);

    /**
     * @brief Generates a collision-free trajectory and tracks it. Returns also possibly
     * the location of a collision on the path if no correct path can be found
     * @param world a pointer to the current world
     * @param field the field object, used onwards by the collision detector
     * @param robotId the ID of the robot for which the path is calculated
     * @param currentPosition the current position of the aforementioned robot
     * @param currentVelocity its velocity
     * @param targetPosition the desired position that the robot has to reach
     * @param targetVelocity the desired velocity that the robot has to reach
     * @param maxRobotVelocity the maximum velocity that the robot is allowed to have
     * @param maxJerk the maximum jerk that the robot is allowed to have
     * @return A RobotCommand and optional with the location of the first collision on the path
     */
    std::pair<Vector2, Vector2> computeAndTrackTrajectory(const rtt::world::World *world, const rtt::Field &field, int robotId, Vector2 currentPosition, Vector2 currentVelocity,
                                                          Vector2 targetPosition, Vector2 targetVelocity, double maxRobotVelocity, double maxJerk, stp::AvoidObjects avoidObjects);

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
     * @param targetPosition the desired position that the robot has to reach
     * @param avoidObjects whether or not to avoid objects
     * @return A new target position
     */
    Vector2 handleBallPlacementCollision(const rtt::world::World *world, const rtt::Field &field, Vector2 currentPosition, Vector2 targetPosition, stp::AvoidObjects avoidObjects);

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
                                   Vector2 &targetPosition, Vector2 &targetVelocity, double maxRobotVelocity, double maxJerk, double timeStep, stp::AvoidObjects AvoidObjects);

    /**
     * @brief Creates normalized random points, which will be used to create intermediate points
     * @param robotId the ID of the robot for which the path is calculated
     * @return A vector with Vector2 of the normalized random points
     */
    std::vector<Vector2> generateNormalizedPoints(int robotId);
};

}  // namespace rtt::ai::control
#endif  // RTT_POSITIONCONTROL_H

//
// Created by martin on 11-5-22.
//

#ifndef RTT_COLLISIONDETECTOR_H
#define RTT_COLLISIONDETECTOR_H

#include <optional>
#include <span>
#include <vector>

#include "PositionControlUtils.h"
#include "control/positionControl/BBTrajectories/BBTrajectory2D.h"
#include "utilities/Constants.h"
#include "world/FieldComputations.h"

namespace rtt::ai::control {

/**
 * @brief This struct represents the obstacles on a field that a robot may encounter at specific time step. It is used in the timeline vector
 */
struct Obstacles {
    StateVector ball; /// Position of the ball
    std::vector<StateVector> robotsThem; /// Position of the enemy robots
    std::unordered_map<int, StateVector> robotsUs; /// Position of our robots. It's a map because we need to know which robot is which (to avoid "collisions" with itself)
};

class CollisionDetector {
   private:
    using RobotView = rtt::world::view::RobotView;
    using BallView = rtt::world::view::BallView;
    Field field;
    std::array<Obstacles, PositionControlUtils::COLLISION_DETECTOR_STEP_COUNT> timeline;

    /**
     * @brief Consolidation of collision detection logic.
     * @tparam pathPoint PathPoint to check for collisions.
     * @param obstaclePos Obstacle to check against.
     * @param minDistance Minimum distance between origin and obstacle.
     * */
    [[nodiscard]] static bool isCollision(const Vector2& position, const Vector2& obstaclePos, double minDistance);

   public:
    CollisionDetector() = default;

    /**
     * @brief This method checks if a robot at a **given position** for a **given time** will collide with any other moving objects (ball / other robots) on the field.
     * @param position Position to check for collisions.
     * @param robotId Robot id to ignore (i.e it self)
     * @param shouldAvoidBall Whether to avoid the ball or not
     * @param timeStep Time step on timeline to check for collisions.
     */
    [[nodiscard]] bool doesCollideWithMovingObjects(const StateVector& state, int robotId, const stp::AvoidObjects& avoidObjects, int timeStep = 0) const;

    /**
     * @brief This method checks if given position is inside the field and outside defense areas.
     * @param position Position to check.
     */
    [[nodiscard]] bool doesCollideWithStaticObjects(const Vector2& position, const stp::AvoidObjects& avoidObjects) const;

    /**
     * @brief Generate the defense area polygons and store them in the class (the computation is really expensive operation)
     */
    void setField(const rtt::Field field);

    /**
     * @brief Updates the timeline with obstacles positions. Must be called *once* at the start of each tick.
     * For enemy robots and the ball => The position is approximated for each time step
     * For our robots                => Only the observed (i.e. current position) is updated
     *                                  If the robot is not moving the current position is set for all time steps.
     * @param robots Robots to update the timeline with
     * @param ball Ball to update the timeline with
     */
    void updateTimeline(const std::vector<RobotView>& robots, const std::optional<BallView>& ball);

    /**
     * @brief Updates the timeline with path approximation generated by path planning.
     * @param path Trajectory approximation.
     * @param currentPosition Current position of the robot.
     * @param robotId Id of robot to update the path for
     */
    void updateTimelineForOurRobot(std::span<const StateVector> path, const Vector2& currentPosition, int robotId);

    /**
     * @brief Draws the timeline to the interface for debugging purposes
     */
    void drawTimeline() const;
};
}  // namespace rtt::ai::control

#endif  // RTT_COLLISIONDETECTOR_H

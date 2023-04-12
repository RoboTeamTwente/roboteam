//
// Created by martin on 14-5-22.
//

#pragma once

#include <variant>

#include "control/positionControl/CollisionDetector.h"
namespace rtt::ai::control {

/**
 * \brief Wrapper container for 2 trajectories and path-score
 * Combining 2 trajectories is an _expensive_ operation, thus it's better to defer that step to the latest possible moment.
 * part1CutoffIndex indicates where endTrajectory connects with initialTrajectory (i.e how many points are used from the initialTrajectory)
 * Lower cost indicated better path (e.g cost = 100 is better than cost = 150)
 */
struct ScoredTrajectoryPair {
    BBTrajectory2D part1;
    BBTrajectory2D part2;
    int part1CutoffIndex = 0;
    int cost = -1;
};

/**
 * Path planning using the BBT algorithm for single robot.
 */
class PathPlanning {
   private:
    // Distance between robot and intermediate point (fieldWidth / RADIUS_OFFSETS)
    static constexpr std::array<int, 3> RADIUS_OFFSETS = {8, 4, 2};
    // How many intermediate points to generate per given radius offset
    static constexpr int POINTS_PER_CIRCLE = 8;
    static constexpr int POINTS_COUNT = RADIUS_OFFSETS.size() * POINTS_PER_CIRCLE;

    // By how much to rotate the intermediate on a given radius offset
    // (so that a no line from the center to the intermediate point overlaps with another)
    static constexpr double ROTATE_OFFSET = M_PI_2;

    // Angle between intermediate points on a given radius offset
    static constexpr double ANGLE_BETWEEN_POINTS = 2 * M_PI / POINTS_PER_CIRCLE;

    const CollisionDetector& collisionDetector;
    double fieldWidth;

    /**
     * \brief Calculates the score of given trajectory. Lower score indicated better path (e.g score = 100 is better than score = 150)
     * @param part1Duration How long the 1st trajectory takes (from ScoredTrajectoryPair)
     * @param part2Duration How long the 2nd trajectory takes (from ScoredTrajectoryPair)
     * @param collision information about *first* collision on the path
     */
    [[nodiscard]] static int scorePath(double part1Duration, double part2Duration, std::optional<double> collisionTime);

    /**
     * \brief Calculates the trajectory of an object and performs collision detection.
     *
     * @param stepOffset The number of steps to offset the trajectory time for collision detection
     *                   (e.g. when generating trajectories that do not originate in current robot position).
     * @param forState State of the robot at the start of the path
     * @param forInput Information from STP about the robot
     * @return A pair consisting of the calculated 2D bang-bang trajectory and an optional collision time.
     */
    [[nodiscard]] std::pair<BBTrajectory2D, std::optional<double>> calculateTrajectoryWithCollisionDetection(int stepOffset, const StateVector& forState, const PositionControlInput& forInput) const;

    /**
     * \brief Find the best trajectory from initial position to the target position that is skewed towards the intermediatePoint.
     * 1) Trajectory is generated from initialPos to intermediatePoint.
     * 2) Trajectory is generated from each path point to the target position.
     * 3) Best trajectory is selected.
     * @param initialPos Initial Position
     * @param initialVel Initial Velocity
     * @param intermediatePoint Is used to generate intermediate trajectory.
     * @param targetPos Target Position
     */
    [[nodiscard]] ScoredTrajectoryPair searchBestTrajectoryToIntermediatePoint(const BBTrajectory2D& directTrajectory, int directTrajectoryCost, const PositionControlInput& input, const Vector2& intermediatePoint) const;

    /**
     * \brief Generates an array of intermediate 2D vectors based on a center point.
     *
     * @param center The center point used to generate the intermediate points.
     * @return An array of `Vector2` objects representing the intermediate points.
     *
     */
    [[nodiscard]] std::array<Vector2, PathPlanning::POINTS_COUNT> generateIntermediatePoints(const Vector2& center) const;

    /**
     * \brief Fills a path buffer with state vectors from a scored trajectory pair.
     *
     * @param pathBuffer The path buffer to fill with state vectors.
     * @param trajectoryPair The scored trajectory pair to extract state vectors from.
     */
    static void fillPathBuffer(std::vector<StateVector>& pathBuffer, const ScoredTrajectoryPair& trajectoryPair);

    /**
     * \brief Fills a path buffer with state vectors based on bang-bang trajectory.
     *
     * @param pathBuffer The path buffer to fill with state vectors.
     * @param trajectory The 2D bang-bang trajectory to generate the state vectors from.
     */
    static void fillPathBuffer(std::vector<StateVector>& pathBuffer, const BBTrajectory2D& trajectory);

   public:
    explicit PathPlanning(const CollisionDetector& collisionDetector);

    /**
     * \brief Computes best trajectory to reach the target position
     * @param initialPos Initial Position
     * @param initialVel Initial Velocity
     * @param targetPos Target Position
     */
    void generateNewPath(std::vector<StateVector>& pathBuffer, const PositionControlInput& input) const noexcept;

    /**
     * \brief Updates the path planning with new information about the world
     * @param field Playing filed
     */
    void updateConstraints(const rtt::Field& field);
};

}  // namespace rtt::ai::control
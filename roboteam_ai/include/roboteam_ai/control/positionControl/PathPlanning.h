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
 * cutoffIndex indicates where endTrajectory connects with initialTrajectory (i.e how many points are used from the initialTrajectory)
 * Lower cost indicated better path (e.g cost = 100 is better than cost = 150)
 */
struct ScoredTrajectoryPair {
    BBTrajectory2D start;
    BBTrajectory2D end;
    int cutoffIndex;
    int cost;
};

/**
 * Path planning using the BBT algorithm for single robot.
 */
class PathPlanning {
   private:
    static constexpr std::array<int, 2> RADIUS_OFFSETS = {8, 2};
    static constexpr int POINTS_PER_CIRCLE = 5;
    static constexpr int POINTS_COUNT = RADIUS_OFFSETS.size() * POINTS_PER_CIRCLE;
    static constexpr double ANGLE_BETWEEN_POINTS = 2 * M_PI / POINTS_PER_CIRCLE;

    const CollisionDetector& collisionDetector;
    double fieldWidth;

    /**
     * \brief Calculates the score of given trajectory. Lower score indicated better path (e.g score = 100 is better than score = 150)
     * @param duration How long the path is (i.e. how many path segments there are in the final path)
     * @param collision information about *first* collision on the path
     */
    [[nodiscard]] static int scorePath(double startTrajectoryDuration, double endTrajectoryDuration, std::optional<double> collisionTime);

    [[nodiscard]] std::pair<BBTrajectory2D, std::optional<double>> trajectoryFromState(int stepOffset, const StateVector& forState, const PositionControlInput& forInput) const;

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
    [[nodiscard]] ScoredTrajectoryPair findTrajectoryForPoint(const BBTrajectory2D directTrajectory, const int directTrajectoryCost, const PositionControlInput& input, const Vector2& intermediatePoint) const;

    /**
     * \brief Generates vector of intermediate points
     * @param center Intermediate points are generated around this point
     */
    [[nodiscard]] std::array<Vector2, PathPlanning::POINTS_COUNT> generateIntermediatePoints(const Vector2& center) const;

    static void fillPathBuffer(std::vector<StateVector>& pathBuffer, const ScoredTrajectoryPair& trajectoryPair);

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
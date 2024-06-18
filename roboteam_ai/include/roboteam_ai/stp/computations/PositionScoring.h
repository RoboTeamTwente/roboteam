#ifndef RTT_POSITIONSCORING_H
#define RTT_POSITIONSCORING_H

#include <roboteam_utils/Field.hpp>

#include "world/World.hpp"

namespace rtt::ai::stp {

namespace gen {
/**
 * @brief Struct that is used to store computations made with this module.
 * Save computations here that are usable for each robot for a position.
 * DO NOT save values specific to a robot in here (like TimeToPosition).
 * If a position's score for a specific evaluation already had been computed in the tick, it will
 * use that value instead of recomputing it. If it was not computed yet, it will compute and save it.
 * @memberof scoreOpen : uint8_t score for the Openness of a position -> evaluations/position/OpennessEvaluation
 * @memberof scoreLineOfSight : uint8_t score for the LineOfSight to a position from a position -> ../LineOfSightEvaluation
 * @memberof scoreGoalShot : uint8_t score for the Goal Shot opportunity for a position -> ../GoalShotEvaluation
 */
struct PositionScores {
    std::optional<double> scoreOpen;
    std::optional<double> scoreLineOfSight;
    std::optional<double> scoreGoalShot;
};

/**
 * @brief Combination of weights for each of the scores.
 * This will be used to determine the final score for a robot for a position.
 * All weights will be multiplied with the corresponding score and then normalized.
 * @memberof weightOpen for scoreOpen
 * @memberof weightLineOfSight for scoreLineOfSight
 * @memberof weightGoalShot for scoreGoalShot
 */
struct ScoreProfile {
    double weightOpen;
    double weightLineOfSight;
    double weightGoalShot;
};

/**
 * @brief Structure with a position and its score
 * @memberof position Vector2 coordinates of a position
 * @memberof score The score for said position
 */
struct ScoredPosition {
    Vector2 position;
    uint8_t score;
};

/**
 * Generalized Position Profiles to be used in plays.
 * They consist of a generalized weight combination.
 */

constexpr ScoreProfile AttackingPass = {0.5, 1, 2}; /**< Scoring weights for Attacking Pass */
constexpr ScoreProfile SafePass = {1, 1, 0};        /**< Scoring weights for Safe Pass, used by the keeper */
constexpr ScoreProfile LineOfSight = {0, 1, 0};     /**< Scoring weights for Line of Sight score, only used for testing minimum line of sight */
constexpr ScoreProfile GoalShot = {0, 0, 1};        /**< Scoring weights for Goal Shot Score, a position where we can shoot at goal */
constexpr ScoreProfile ChippingPass = {1, 0, 1};    /**< Scoring weights for ChippingPass score */
}  // namespace gen

/**
 * @brief Class that manages the scoring for a position
 */
class PositionScoring {
   private:
    /**
     * @brief Determine score for the Open score at given position
     * @param point Position to calculate from
     * @param field The current field
     * @param world The current world
     * @param scores ref to struct linked to that pos
     * @return Open score value
     */
    static double determineOpenScore(Vector2 &point, const rtt::Field &field, const world::World *world, gen::PositionScores &scores);

    /**
     * @brief Determine score for the Line of Sight to the ball at given position
     * @param point Position to calculate from
     * @param world The current world
     * @param scores ref to struct linked to that pos
     * @return Line of Sight score value
     */
    static double determineLineOfSightScore(Vector2 &point, const world::World *world, gen::PositionScores &scores);

    /**
     * @brief Determine score for the Visibility of the goal at given position
     * @param point Position to calculate from
     * @param field The current field
     * @param world The current world
     * @param scores ref to struct linked to that pos
     * @return Goal Shot score value
     */
    static double determineGoalShotScore(Vector2 &point, const Field &field, const world::World *world, gen::PositionScores &scores);

    /**
     * @brief Score a position using the given weights weights for a profile.
     * Will check if the position already has a pre-calculated score (from this tick) then throws the weight over it
     * and sums the scores, resulting in a position scored a particular set of weights.
     * @param profile set of weights of the different factors that determine the score
     * @param position x,y coordinates
     * @param scores reference to scores of said position in map
     * @param field The current field
     * @param world The current world
     * @return score of position including the weights
     */
    static uint8_t getScoreOfPosition(const gen::ScoreProfile &profile, Vector2 position, gen::PositionScores &scores, const Field &field, const world::World *world);

   public:
    /**
     * @brief Get score of a position, used in getPosition
     * @param position Vector2 that needs to be scored
     * @param profile combination of weights for different factors that should be scored
     * @param field The current field
     * @param world The current world
     * @param bias value added to score
     * @return Position with score
     */
    static gen::ScoredPosition scorePosition(const Vector2 &position, const gen::ScoreProfile &profile, const Field &field, const world::World *world, uint8_t bias = 0);
};
}  // namespace rtt::ai::stp
#endif  // RTT_POSITIONSCORING_H

//
// Created by maxl on 19-03-21.
//

#ifndef RTT_GENERALIZATIONCONSTANTS_H
#define RTT_GENERALIZATIONCONSTANTS_H

#include <roboteam_utils/Grid.h>

#include <optional>
#include <unordered_map>

namespace rtt::ai::stp::gen {
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

constexpr ScoreProfile AttackingPass = {0.5, 1, 1}; /**< Scoring weights for Attacking Pass */
constexpr ScoreProfile SafePass = {1, 1, 0}; /**< Scoring weights for Safe Pass, used by the keeper */
constexpr ScoreProfile LineOfSight = {0, 1, 0}; /**< Scoring weights for Line of Sight score, only used for testing minimum line of sight */
constexpr ScoreProfile GoalShot = {0, 0, 1}; /**< Scoring weights for Goal Shot Score, a position where we can shoot at goal */
constexpr ScoreProfile ChippingPass = {1, 0, 1}; /**< Scoring weights for ChippingPass score */

}  // namespace rtt::ai::stp::gen
#endif  // RTT_GENERALIZATIONCONSTANTS_H

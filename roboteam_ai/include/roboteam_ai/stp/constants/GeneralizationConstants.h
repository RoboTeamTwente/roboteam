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
 * @memberof scoreBlocking : uint8_t score for the potential to block from a position -> ../BlockingEvaluation
 */
struct PositionScores {
    std::optional<double> scoreOpen;
    std::optional<double> scoreLineOfSight;
    std::optional<double> scoreGoalShot;
    std::optional<double> scoreBlocking;
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
    double weightBlocking;
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
constexpr ScoreProfile SafePosition = {1, 1, 0, 0.5}; /**< Scoring weights for Safe Position */
constexpr ScoreProfile OffensivePosition = {1, 0.5, 0.5, 0}; /**< Scoring weights for Offensive Position */
constexpr ScoreProfile BlockingPosition = {0, 0.5, 0, 1}; /**< Scoring weights for Blocking Positions */
constexpr ScoreProfile AttackingPass = {0.5, 1, 1, 0}; /**< Scoring weights for Attacking Pass */
constexpr ScoreProfile SafePass = {1, 1, 0.5, 0}; /**< Scoring weights for Safe Pass */
constexpr ScoreProfile LineOfSight = {0, 1, 0, 0}; /**< Scoring weights for Line of Sight score */
constexpr ScoreProfile Open = {1, 0, 0, 0}; /**< Scoring weights for Open score */
constexpr ScoreProfile GoalShot = {0, 0, 1, 0}; /**< Scoring weights for Goal Shot Score */

/**
 * @brief Generalized Keys for passing information form the old play to the new.
 * Usage in the storePlayInfo where KeyInfo is the key for the elements in the map.
 */
enum class KeyInfo {
    hasBall /**< Robot that had the ball last play */
};

/**
 * @brief Generalized information structure for the map of storePlayInfo.
 * Allows for saving specific information from the old play to the new.
 */
struct StoreInfo {
    std::optional<int> robotID;  /**< ID of the robot */
    std::optional<Vector2> robotPosition; /**< Current position of the robot */
    std::optional<Vector2> moveToPosition; /**< Position the robot should move to */
    std::optional<Vector2> defendPosition; /**< Position the robot should defend */
    std::optional<Vector2> shootAtPosition; /**< Position the robot should shoot at */
    std::optional<Vector2> passToRobot; /**< Position of a robot the robot should pass to */
};

using PlayInfos = std::unordered_map<KeyInfo, StoreInfo>; /**< Place to store info in that is needed between Plays. Used in storePlayInfo. */

}  // namespace rtt::ai::stp::gen
#endif  // RTT_GENERALIZATIONCONSTANTS_H

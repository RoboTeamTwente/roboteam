#ifndef RTT_CONTROLCONSTANTS_H
#define RTT_CONTROLCONSTANTS_H

#include <math.h>

#include <cstdint>

namespace rtt::ai::stp::control_constants {

// Kick and chip constants
constexpr double MAX_KICK_POWER = 6.5; /**< Maximum allowed kicking power */
constexpr double MIN_KICK_POWER = 3.0; /**< Minimum allowed kicking power */
constexpr double MAX_CHIP_POWER = 6.5; /**< Maximum allowed chipping power */
constexpr double MIN_CHIP_POWER = 4.5; /**< Minimum allowed chipping power */
// Max attempts before the force_kick_chip is set to true
constexpr double MAX_KICK_ATTEMPTS = 25; /**< Maximum allowed kicking attempts */
constexpr double MAX_CHIP_ATTEMPTS = 25; /**< Maximum allowed chipping attempts */

/// Robot physical constants
constexpr double ROBOT_RADIUS = 0.088;    /**< Radius of a robot */
constexpr double CENTER_TO_FRONT = 0.069; /**< Distance from center of the robot to the front of the robot */

/// Dribbler constants
// The distance from robot to ball at which the dribbler should turn on
constexpr double TURN_ON_DRIBBLER_DISTANCE = 5 * ROBOT_RADIUS; /**< Distance from a robot to the ball at which the dribbler should turn on */

constexpr uint8_t MAX_ROBOT_COUNT = 11; /**< Maximum allowed number of robots */

/// Ball constants
constexpr double BALL_STILL_VEL = 0.1;                              /**< Velocity of the ball at which it is considered still */
constexpr double BALL_STILL_VEL2 = BALL_STILL_VEL * BALL_STILL_VEL; /**< Squared velocity of the ball at which it is considered still */
constexpr double BALL_GOT_SHOT_LIMIT = 0.6;                         /**< Velocity of the ball at which it is considered shot */
constexpr double BALL_IS_MOVING_SLOW_LIMIT = 1;                     /**< Velocity of the ball at which it is considered moving slow */
constexpr double BALL_RADIUS = 0.0215;                              /**< Radius of the ball */
constexpr double HAS_CHIPPED_ERROR_MARGIN = 0.4;                    /**< Velocity margin for detecting ball chips */
constexpr double ENEMY_CLOSE_TO_BALL_DISTANCE = 1.0;                /**< Distance from the ball to an enemy robot at which the ball is considered close to the enemy */

/// RobotCommand limits
constexpr double MAX_DRIBBLER_CMD = 1; /**< Maximum allowed dribbler velocity that can be send to the robot */
// Yaw increment per tick
constexpr double YAW_RATE = 0.2 * M_PI;       /**< Maximum allowed angular velocity that can be send to the robot */
constexpr double MAX_VEL_WHEN_HAS_BALL = 3.0; /**< Maximum allowed velocity that can be send to the robot when that robot has the ball */

/// GoToPos Constants
// Distance margin for 'goToPos'. If the robot is within this margin, goToPos is successful
constexpr double GO_TO_POS_ERROR_MARGIN = 0.01; /**< Distance error for a robot to be considered to have reached a position */
// Angle margin for 'goToPos'. If the robot is within this margin, goToPos is successful
constexpr double GO_TO_POS_ANGLE_ERROR_MARGIN = 0.01; /**< Angle error for a robot to be considered to have reached a position */
// Maximum inaccuracy during ballplacement
constexpr double BALL_PLACEMENT_MARGIN = 0.15 - BALL_RADIUS - 0.02; /**< Distance error for the ball to be considered to have reached the ball placement position*/
constexpr double ENEMY_ALREADY_ASSIGNED_MULTIPLIER = 0.9;           /**< Multiplication factor for the distance to goal used by the dealer when the enemy is already assigned */

/// Invariant constants
constexpr uint8_t FUZZY_TRUE = 255;          /**< Value at which the fuzzy logic is considered 100% true */
constexpr uint8_t FUZZY_FALSE = 0;           /**< Value at which the fuzzy logic is considered 0% true */
constexpr double FUZZY_DEFAULT_CUTOFF = 127; /**< Value at which the fuzzy logic is considered 50% true */

/// Distance constants
constexpr double ROBOT_CLOSE_TO_POINT = 0.2;        /**< Distance from the robot to a position at which the robot is considered close to that position */
constexpr double DISTANCE_TO_PASS_TRAJECTORY = 0.5; /**< Distance from the robot to the pass trajectory at which the robot is considered too close to the pass trajectory */
constexpr double OUT_OF_FIELD_MARGIN = 0.17;      /**< Distance that the center of the robot is allowed to go out of the field during play (not for end location, only for paths) */
constexpr double FREE_KICK_TAKEN_DISTANCE = 0.07; /**< Distance that the ball must travel before we can say a kickoff or free kick has been taken */

/// GameState constants
constexpr double AVOID_BALL_DISTANCE = 0.5 + ROBOT_RADIUS + GO_TO_POS_ERROR_MARGIN + BALL_RADIUS + 0.1; /**< Minimum distance all robots should keep when avoiding the ball */
constexpr double AVOID_BALL_DISTANCE_BEFORE_FREE_KICK =
    0.05 + ROBOT_RADIUS + GO_TO_POS_ERROR_MARGIN + BALL_RADIUS + 0.01; /**< Minimum distance all robots should keep when avoiding the ball before a free kick */

/// Friction constants
constexpr static float SIMULATION_FRICTION = 0.71; /**< The expected movement friction of the ball during simulation */
constexpr static float REAL_FRICTION = 0.44;       /**< The expected movement friction of the ball on the field */

}  // namespace rtt::ai::stp::control_constants

#endif  // RTT_CONTROLCONSTANTS_H

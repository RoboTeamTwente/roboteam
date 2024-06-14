#ifndef ROBOTEAM_AI_CONSTANTS_H
#define ROBOTEAM_AI_CONSTANTS_H

#include <math.h>

#include <cstdint>

#include "utilities/GameSettings.h"

namespace rtt::ai::constants {

constexpr bool FEEDBACK_ENABLED = true; /**< Checks whether robot feedback is enabled */
// Kick and chip constants
constexpr double MAX_KICK_POWER = 6.5; /**< Maximum allowed kicking power */
constexpr double MIN_KICK_POWER = 3.0; /**< Minimum allowed kicking power */
constexpr double MAX_CHIP_POWER = 6.5; /**< Maximum allowed chipping power */
constexpr double MIN_CHIP_POWER = 4.5; /**< Minimum allowed chipping power */

/// Robot physical constants
constexpr double ROBOT_RADIUS = 0.089;    /**< Radius of a robot */
constexpr double CENTER_TO_FRONT = 0.069; /**< Distance from center of the robot to the front of the robot */

/// Dribbler constants
// The distance from robot to ball at which the dribbler should turn on
constexpr double TURN_ON_DRIBBLER_DISTANCE = 5 * ROBOT_RADIUS; /**< Distance from a robot to the ball at which the dribbler should turn on */
constexpr uint8_t MAX_ROBOT_COUNT = 11;                        /**< Maximum allowed number of robots */
constexpr uint64_t WORLD_MAX_AGE_MILLISECONDS = 1000;          /**< Maximum amount of time the AI can run without receiving a new world update */
constexpr int STP_TICK_RATE = 60;                              /**< The rate at which the STP runs in ticks per second */
constexpr int SETTING_BROADCAST_RATE = 1;                      /**< The rate at which the AI broadcasts its settings */

/// Ball constants
constexpr double BALL_STILL_VEL = 0.1;                              /**< Velocity of the ball at which it is considered still */
constexpr double BALL_STILL_VEL2 = BALL_STILL_VEL * BALL_STILL_VEL; /**< Squared velocity of the ball at which it is considered still */
constexpr double BALL_GOT_SHOT_LIMIT = 0.6;                         /**< Velocity of the ball at which it is considered shot */
constexpr double BALL_IS_MOVING_SLOW_LIMIT = 1;                     /**< Velocity of the ball at which it is considered moving slow */
constexpr double BALL_RADIUS = 0.0215;                              /**< Radius of the ball */
constexpr double HAS_CHIPPED_ERROR_MARGIN = 0.4;                    /**< Velocity margin for detecting ball chips */
constexpr double ENEMY_CLOSE_TO_BALL_DISTANCE = 1.0;                /**< Distance from the ball to an enemy robot at which the ball is considered close to the enemy */
constexpr double HAS_BALL_ANGLE = 0.1 * M_PI;                       /**< Maximum angle between the robot and the ball at which the robot is considered to have the ball */

// Yaw increment per tick
constexpr double YAW_RATE = 0.2 * M_PI;       /**< Maximum allowed angular velocity that can be send to the robot */
constexpr double MAX_VEL_WHEN_HAS_BALL = 3.0; /**< Maximum allowed velocity that can be send to the robot when that robot has the ball */
constexpr double MAX_ANGULAR_VELOCITY = 6.0;  /**< Maximum allowed angular velocity that can be send to the robot */
constexpr double MIN_YAW = -M_PI;             /**< Minimum yaw the robot can have */
constexpr double MAX_YAW = M_PI;              /**< Maximum yaw the robot can have */
constexpr double MAX_ACC = 3.5;               /**< Maximum acceleration of the robot */
constexpr double MAX_VEL = 4.0;               /**< Maximum allowed velocity of the robot */

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
constexpr double PENALTY_DISTANCE_BEHIND_BALL = 1.5; /**< The minimum distance the robots have to be behind the ball during a penalty, default is 1 meter, but do 1.5 to be sure */
;

/// GameState constants
constexpr double AVOID_BALL_DISTANCE = 0.5 + ROBOT_RADIUS + GO_TO_POS_ERROR_MARGIN + BALL_RADIUS + 0.1; /**< Minimum distance all robots should keep when avoiding the ball */
constexpr double AVOID_BALL_DISTANCE_BEFORE_FREE_KICK =
    0.05 + ROBOT_RADIUS + GO_TO_POS_ERROR_MARGIN + BALL_RADIUS + 0.01; /**< Minimum distance all robots should keep when avoiding the ball before a free kick */

/// Friction constants
constexpr static float SIMULATION_FRICTION = 0.71; /**< The expected movement friction of the ball during simulation */
constexpr static float REAL_FRICTION = 0.44;       /**< The expected movement friction of the ball on the field */

static inline double HAS_BALL_DISTANCE() { return (GameSettings::getRobotHubMode() == net::RobotHubMode::BASESTATION) ? 0.11 : 0.12; }

static std::map<int, bool> ROBOTS_WITH_WORKING_DRIBBLER() {
    static std::map<int, bool> workingDribblerRobots;
    workingDribblerRobots[0] = true;
    workingDribblerRobots[1] = true;
    workingDribblerRobots[2] = true;
    workingDribblerRobots[3] = true;
    workingDribblerRobots[4] = true;
    workingDribblerRobots[5] = true;
    workingDribblerRobots[6] = true;
    workingDribblerRobots[7] = true;
    workingDribblerRobots[8] = true;
    workingDribblerRobots[9] = true;
    workingDribblerRobots[10] = true;
    workingDribblerRobots[11] = true;
    workingDribblerRobots[12] = true;
    workingDribblerRobots[13] = true;
    workingDribblerRobots[14] = true;
    workingDribblerRobots[15] = true;

    return workingDribblerRobots;
}

static std::map<int, bool> ROBOTS_WITH_WORKING_BALL_SENSOR() {
    static std::map<int, bool> workingBallSensorRobots;
    workingBallSensorRobots[0] = false;
    workingBallSensorRobots[1] = false;
    workingBallSensorRobots[2] = false;
    workingBallSensorRobots[3] = false;
    workingBallSensorRobots[4] = false;
    workingBallSensorRobots[5] = false;
    workingBallSensorRobots[6] = false;
    workingBallSensorRobots[7] = false;
    workingBallSensorRobots[8] = false;
    workingBallSensorRobots[9] = false;
    workingBallSensorRobots[10] = false;
    workingBallSensorRobots[11] = false;
    workingBallSensorRobots[12] = false;
    workingBallSensorRobots[13] = false;
    workingBallSensorRobots[14] = false;
    workingBallSensorRobots[15] = false;

    return workingBallSensorRobots;
}

static std::map<int, bool> ROBOTS_WITH_WORKING_DRIBBLER_ENCODER() {
    static std::map<int, bool> workingDribblerEncoderRobots;
    workingDribblerEncoderRobots[0] = true;
    workingDribblerEncoderRobots[1] = true;
    workingDribblerEncoderRobots[2] = true;
    workingDribblerEncoderRobots[3] = true;
    workingDribblerEncoderRobots[4] = true;
    workingDribblerEncoderRobots[5] = true;
    workingDribblerEncoderRobots[6] = true;
    workingDribblerEncoderRobots[7] = true;
    workingDribblerEncoderRobots[8] = true;
    workingDribblerEncoderRobots[9] = true;
    workingDribblerEncoderRobots[10] = true;
    workingDribblerEncoderRobots[11] = true;
    workingDribblerEncoderRobots[12] = true;
    workingDribblerEncoderRobots[13] = true;
    workingDribblerEncoderRobots[14] = true;
    workingDribblerEncoderRobots[15] = true;

    return workingDribblerEncoderRobots;
}

static std::map<int, bool> ROBOTS_WITH_KICKER() {
    static std::map<int, bool> kickerRobots;
    kickerRobots[0] = true;
    kickerRobots[1] = true;
    kickerRobots[2] = true;
    kickerRobots[3] = true;
    kickerRobots[4] = true;
    kickerRobots[5] = true;
    kickerRobots[6] = true;
    kickerRobots[7] = true;
    kickerRobots[8] = true;
    kickerRobots[9] = true;
    kickerRobots[10] = true;
    kickerRobots[11] = true;
    kickerRobots[12] = true;
    kickerRobots[13] = true;
    kickerRobots[14] = true;
    kickerRobots[15] = true;

    return kickerRobots;
}

[[maybe_unused]] static bool ROBOT_HAS_WORKING_BALL_SENSOR(int id) { return ROBOTS_WITH_WORKING_BALL_SENSOR()[id]; }

[[maybe_unused]] static bool ROBOT_HAS_WORKING_DRIBBLER(int id) { return ROBOTS_WITH_WORKING_DRIBBLER()[id]; }

[[maybe_unused]] static bool ROBOT_HAS_WORKING_DRIBBLER_ENCODER(int id) { return ROBOTS_WITH_WORKING_DRIBBLER_ENCODER()[id]; }

[[maybe_unused]] static bool ROBOT_HAS_KICKER(int id) { return ROBOTS_WITH_KICKER()[id]; }

}  // namespace rtt::ai::constants

#endif  // RTT_CONSTANTS_H

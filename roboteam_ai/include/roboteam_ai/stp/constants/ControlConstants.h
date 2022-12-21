/**
 * @file ControlConstants.h
 * @author jessevw
 * @brief All constants used for robot control within AI
 */
#ifndef RTT_CONTROLCONSTANTS_H
#define RTT_CONTROLCONSTANTS_H

#include <cstdint>

namespace rtt::ai::stp::control_constants {

extern const double MAX_KICK_POWER; /**< Maximum allowed kicking power */
extern const double MIN_KICK_POWER; /**< Minimum allowed kicking power */
extern const double MAX_CHIP_POWER; /**< Maximum allowed chipping power */
extern const double MIN_CHIP_POWER; /**< Minimum allowed chipping power */
extern const double MAX_POWER_KICK_DISTANCE; /**< Distance at which the kicking power should be at maximum */
extern const double MAX_POWER_CHIP_DISTANCE; /**< Distance at which the chipping power should be at maximum */
extern const double MAX_KICK_ATTEMPTS; /**< Maximum allowed kicking attempts */
extern const double MAX_CHIP_ATTEMPTS; /**< Maximum allowed chipping attempts */
extern const double ROBOT_RADIUS; /**< Radius of a robot */
extern const double CENTER_TO_FRONT; /**< Distance from center of the robot to the front of the robot */
extern const double TURN_ON_DRIBBLER_DISTANCE; /**< Distance from a robot to the ball at which the dribbler should turn on */
inline constexpr uint8_t MAX_ROBOT_COUNT = 11; /**< Maximum allowed number of robots */
extern const double BALL_STILL_VEL; /**< Velocity of the ball at which it is considered still */
extern const double BALL_STILL_VEL2; /**< Squared velocity of the ball at which it is considered still */
extern const double BALL_GOT_SHOT_LIMIT; /**< Velocity of the ball at which it is considered shot */
extern const double BALL_IS_MOVING_SLOW_LIMIT; /**< Velocity of the ball at which it is considered moving slow */
extern const double BALL_IS_CLOSE; /**< Distance from the ball to a robot at which the ball is considered close */
extern const double BALL_RADIUS; /**< Radius of the ball */
extern const double HAS_KICKED_ERROR_MARGIN; /**< Velocity margin for detecting ball kicks */
extern const double HAS_CHIPPED_ERROR_MARGIN; /**< Velocity margin for detecting ball chips */
extern const double ENEMY_CLOSE_TO_BALL_DISTANCE; /**< Distance from the ball to an enemy robot at which the ball is considered close to the enemy */
extern const double MAX_VEL_CMD; /**< Maximum allowed velocity that can be send to the robot */
extern const double MAX_DRIBBLER_CMD; /**< Maximum allowed dribbler velocity that can be send to the robot */
extern const double ANGLE_RATE; /**< Maximum allowed angular velocity that can be send to the robot */
extern const double MAX_VEL_WHEN_HAS_BALL; /**< Maximum allowed velocity that can be send to the robot when that robot has the ball */
extern const double HAS_BALL_ANGLE_ERROR_MARGIN; /**< Angle error for detecting whether a robot has the ball */
extern const double HAS_BALL_DISTANCE_ERROR_MARGIN; /**< Distance error for detecting whether a robot has the ball */
extern const double GO_TO_POS_ERROR_MARGIN; /**< Distance error for a robot to be considered to have reached a position */
extern const double GO_TO_POS_ANGLE_ERROR_MARGIN; /**< Angle error for a robot to be considered to have reached a position */
extern const double BALL_PLACEMENT_MARGIN; /**< Distance error for the ball to be considered to have reached the ball placement position*/
extern const uint8_t FUZZY_TRUE; /**< Value at which the fuzzy logic is considered 100% true */
extern const uint8_t FUZZY_FALSE; /**< Value at which the fuzzy logic is considered 0% true */
extern const double FUZZY_MARGIN; /**< Error margin of the fuzzy logic */
extern const double FUZZY_DEFAULT_CUTOFF; /**< Value at which the fuzzy logic is considered 50% true */
extern const double DISTANCE_TO_ROBOT_CLOSE; /**< Distance from the robot to another robot at which the robot is considered close to that other robot */
extern const double DISTANCE_TO_ROBOT_FAR; /**< Distance from the robot to another robot at which the robot is considered far from that other robot */
extern const double ROBOT_CLOSE_TO_POINT; /**< Distance from the robot to a position at which the robot is considered close to that position */
extern const double DISTANCE_TO_ROBOT_NEAR; /**< Distance from the robot to another robot at which the robot is considered near that other robot */
extern const double DEFENSE_AREA_AVOIDANCE_MARGIN; /**< Distance error for avoiding the defense area */
extern const double DISTANCE_FROM_GOAL_CLOSE; /**< Distance from the keeper to the goal at which the keeper is considered close to that goal */
extern const double AVOID_BALL_DISTANCE; /**< Minimum distance all robots should keep when avoiding the ball */

}  // namespace rtt::ai::stp::control_constants

#endif  // RTT_CONTROLCONSTANTS_H

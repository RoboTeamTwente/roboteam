//
// Created by alexander on 03-12-21.
//

#ifndef RTT_STPINFOENUMS_H
#define RTT_STPINFOENUMS_H

#include "stp/constants/ControlConstants.h"

namespace rtt::ai::stp {
/**
 * @brief The distance the robot should block at
 */
enum class BlockDistance { ROBOTRADIUS, CLOSE, PARTWAY, HALFWAY, FAR };

/**
 * @brief Whether this robot should kick or chip in the shoot skill
 */
enum class KickOrChip { KICK, CHIP };

/**
 * @brief The PIDType to be used by this robot for path planning/tracking
 */
enum class PIDType { DEFAULT, RECEIVE, INTERCEPT, KEEPER, KEEPER_INTERCEPT };

/**
 * @brief The type of shot this robot should use. Used for determining kick/chip velocity
 */
enum class ShotType { PASS, TARGET, MAX };

/**
 * @brief The status that a skill/tactic can return
 */
enum class Status { Waiting, Success, Failure, Running };

/**
 * @brief The AvoidObjects struct containing what the robot should avoid
 */
struct AvoidObjects {
    bool shouldAvoidBall = false; /***< Indicates whether the robot should avoid the ball */
    bool shouldAvoidDefenseArea = true; /***< Indicates whether the robot should avoid the defense area */
    bool shouldAvoidOutOfField = true; /***< Indicates whether the robot should avoid going out of the field */
    bool shouldAvoidOurRobots = true; /***< Indicates whether the robot should avoid allied robots */
    bool shouldAvoidTheirRobots = true; /***< Indicates whether the robot should avoid the enemy robots */
    double avoidBallDist = 2.0 * stp::control_constants::ROBOT_RADIUS; /***< The minimum distance a robot should keep to the ball when avoiding the ball */
};
}  // namespace rtt::ai::stp
#endif  // RTT_STPINFOENUMS_H

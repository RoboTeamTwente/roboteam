#ifndef RTT_STPINFOENUMS_H
#define RTT_STPINFOENUMS_H

#include <utilities/Constants.h>
namespace rtt::ai::stp {
/**
 * @brief The type of shot this robot should use. Used for determining kick/chip velocity
 */
enum class ShotPower { PASS, TARGET, MAX };

/**
 * @brief The status that a skill/tactic can return
 */
enum class Status { Waiting, Success, Failure, Running };

/**
 * @brief The AvoidObjects struct containing what the robot should avoid
 */
struct AvoidObjects {
    bool shouldAvoidGoalPosts = true;         /***< Indicates whether the robot should avoid the goal posts */
    bool shouldAvoidOutOfField = true;        /***< Indicates whether the robot should avoid going out of the field */
    bool shouldAvoidOurDefenseArea = false;   /***< Indicates whether the robot should avoid the defense area of the team */
    bool shouldAvoidTheirDefenseArea = false; /***< Indicates whether the robot should avoid the defense area of the enemy team */
    bool shouldAvoidOurRobots = true;         /***< Indicates whether the robot should avoid allied robots */
    bool shouldAvoidTheirRobots = true;       /***< Indicates whether the robot should avoid the enemy robots */
    bool shouldAvoidBall = false;             /***< Indicates whether the robot should avoid the ball */
};
}  // namespace rtt::ai::stp
#endif  // RTT_STPINFOENUMS_H

#ifndef ROBOTEAM_AI_CONSTANTS_H
#define ROBOTEAM_AI_CONSTANTS_H

#include <map>
#include <vector>

#include "RuleSet.h"
#include "math.h"

namespace rtt::ai {

/**
 * @brief Class that defines the constants used in this code.
 * These constants are stored here so that we don't have to redefine them everytime we use them. Having them in one place also gives a lot of consistency.
 */
class Constants {
   public:
    static void init();             /**< Initializes the constants for either the simulator or basestation. */
    static bool FEEDBACK_ENABLED(); /**< Checks whether robot feedback is enabled */

    /**
     * @brief Indicates the maximum amount of time the AI can run without receiving a new world update
     * @return The maximum amount of time
     */
    static constexpr uint64_t WORLD_MAX_AGE_MILLISECONDS() { return 1000; }

    /**
     * @brief Checks the amount of robots present
     * @return The amount of robots
     */
    static constexpr size_t ROBOT_COUNT() { return 11; };

    /**
     * @brief Checks the speed at which STP is running in ticks per second
     * @return The tick rate of STP
     */
    static constexpr int STP_TICK_RATE() { return 60; };

    /**
     * @brief Checks at which rate the AI should broadcast its settings
     * @return The rate at which AI broadcasts its settings
     */
    static constexpr int SETTINGS_BROADCAST_RATE() { return 1; }

    /// ROBOT COMMANDS ///
    static double MAX_VEL_CMD();          /**< The maximum allowed velocity of the robot */
    static double MIN_YAW();              /**< The minimum yaw the robot can have */
    static double MAX_YAW();              /**< The maximum yaw the robot can have */
    static double MAX_ANGULAR_VELOCITY(); /**< The maximum angular velocity of the robot */
    static double MAX_ACC();              /**< Maximum acceleration of the robot */

    /// ROBOT GEOMETRY ///
    static constexpr double ROBOT_RADIUS() { return 0.089; }; /**< Radius of the robot */
    static constexpr double BALL_RADIUS() { return 0.0215; }; /**< Radius of the ball */
    static double HAS_BALL_DISTANCE();                        /** The distance at which the robot is considered to have the ball */
    static double HAS_BALL_ANGLE();                           /** The angle at which the robot is considered to have the ball */

    /// REF STATES ///
    static constexpr double MAX_VEL() { return 4.0; }; /**< Maximum allowed velocity */
    static int DEFAULT_KEEPER_ID();                    /**< Default ID of the keeper */
    static double PENALTY_DISTANCE_BEHIND_BALL();      /**< The minimum distance the robots have to be behind the ball during a penalty */

    static std::map<int, bool> ROBOTS_WITH_WORKING_DRIBBLER();         /**< Mapping of robots with working dribblers */
    static std::map<int, bool> ROBOTS_WITH_WORKING_BALL_SENSOR();      /**< Mapping of robots with working ball sensors */
    static std::map<int, bool> ROBOTS_WITH_WORKING_DRIBBLER_ENCODER(); /**< Mapping of robots with working dribbler encoders */
    static std::map<int, bool> ROBOTS_WITH_KICKER();                   /**< Mapping of robots with working kicker */
    static std::map<int, float> ROBOTS_MAXIMUM_KICK_TIME();            /**< Mapping robots with their respective time to charge the kicker */

    static bool ROBOT_HAS_WORKING_DRIBBLER(int id);         /**< Checks whether the given robot has a working dribbler */
    static bool ROBOT_HAS_WORKING_BALL_SENSOR(int id);      /**< Checks whether the given robot has a working ball sensor */
    static bool ROBOT_HAS_WORKING_DRIBBLER_ENCODER(int id); /**< Checks whether the given robot has a working dribbler encoder */
    static bool ROBOT_HAS_KICKER(int id);                   /**< Checks whether the given robot has a working kicker */

    static RuleSet RULESET_DEFAULT();
    static RuleSet RULESET_HALT();
    static RuleSet RULESET_STOP();

    /**
     * @brief Returns a vector of all the predefined rulesets
     * @return a vector of all the predefined rulesets
     */
    static std::vector<RuleSet> ruleSets();
};

}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_CONSTANTS_H

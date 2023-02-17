
#ifndef ROBOTEAM_AI_CONSTANTS_H
#define ROBOTEAM_AI_CONSTANTS_H

#include <QColor>
#include <map>
#include <vector>

#include "RuleSet.h"
#include "math.h"

namespace rtt::ai {

typedef std::tuple<double, double, double> pidVals; /**< Defenition of the pidVals tuple. This is used for storing and using pid values. Do not ask me why it has to be a tuple. */

/**
 * @brief Class that defines the constants used in this code.
 * These constants are stored here so that we don't have to redefine them everytime we use them. Having them in one place also gives a lot of consistency.
 */
class Constants {
   public:
    static bool FEEDBACK_ENABLED(); /**< Checks whether robot feedback is enabled */

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

    /**
     * @brief Checks which rules apply for the current game state
     * @return The ruleset according to the current game state
     */
    static std::vector<RuleSet> ruleSets();

    /// ROBOT COMMANDS ///
    static double MAX_VEL_CMD(); /**< The maximum allowed velocity of the robot */
    static double MIN_ANGLE(); /**< The minimum angle the robot can have */
    static double MAX_ANGLE(); /**< The maximum angle the robot can have */
    static double MAX_ANGULAR_VELOCITY(); /**< The maximum angular velocity of the robot */
    static double MAX_ACC_UPPER(); /**< Maximum acceleration for moving in the forward direction */
    static double MAX_ACC_LOWER(); /**< Maximum acceleration for moving in the sideways direction */
    static double MAX_DEC_UPPER(); /**< Maximum deceleration for moving in the forward direction */
    static double MAX_DEC_LOWER(); /**< Maximum deceleration for moving in the sideways direction */

    /// ROBOT GEOMETRY ///
    static constexpr double ROBOT_RADIUS() { return 0.089; }; /**< Radius of the robot */
    static constexpr double ROBOT_RADIUS_MAX() { return 0.091; }; /**< The maximum radius any robot can have */
    static constexpr double BALL_RADIUS() { return 0.0215; }; /**< Radius of the ball */
    static double HAS_BALL_DISTANCE(); /** The distance at which the robot is considered to have the ball */
    static double HAS_BALL_ANGLE(); /** The angle at which the robot is considered to have the ball */

    /// REF STATES ///
    static constexpr double MAX_VEL() { return 1.5; }; /**< Maximum allowed velocity */
    static int DEFAULT_KEEPER_ID(); /**< Default ID of the keeper */
    static double PENALTY_DISTANCE_BEHIND_BALL(); /**< The minimum distance the robots have to be behind the ball during a penalty */

    /// INTERFACE ///
    static int ROBOT_DRAWING_SIZE(); /**< Size the robots should drawn with */
    static int BALL_DRAWING_SIZE(); /**< Size the ball should be drawn with */
    static int TACTIC_COLOR_DRAWING_SIZE(); /**< Size the tactic color should be drawn with */
    static int WINDOW_FIELD_MARGIN(); /**< The margin between the field and the window borders */
    static QColor FIELD_COLOR(); /**< The color of the field */
    static QColor FIELD_LINE_COLOR(); /**< The color of the field lines */
    static QColor ROBOT_COLOR_BLUE(); /**< The color for the blue robots */
    static QColor ROBOT_COLOR_YELLOW(); /**< The color for the yellow robots */
    static QColor BALL_COLOR(); /**< The color of the ball */
    static QColor TEXT_COLOR(); /**< The color of text */
    static QColor SELECTED_ROBOT_COLOR(); /**< The color of the currently selected robot */
    static std::vector<QColor> TACTIC_COLORS(); /**< The color of the tactics */

    /// SETTINGS ///
    static bool STD_SHOW_ROLES(); /**< Checks whether the roles should be shown */
    static bool STD_SHOW_TACTICS(); /**< Checks whether the tactics should be shown */
    static bool STD_SHOW_TACTICS_COLORS(); /**< Checks whether the tactic colors should be shown */
    static bool STD_SHOW_VELOCITIES(); /**< Checks whether the velocities should be shown */
    static bool STD_SHOW_ANGLES(); /**< Checks whether the angles should be shown */
    static bool STD_SHOW_ROBOT_INVALIDS(); /**< Checks whether the robot invalids should be shown */
    static bool STD_SHOW_BALL_PLACEMENT_MARKER(); /**< Checks whether the ball placement marker should be shown */
    static bool STD_USE_REFEREE(); /**< Checks whether the referee should be use */
    static bool STD_TIMEOUT_TO_TOP(); /**< Checks whether the robots should move to the top during timeout */

    static std::map<int, bool> ROBOTS_WITH_WORKING_DRIBBLER(); /**< Mapping of robots with working dribblers */
    static std::map<int, bool> ROBOTS_WITH_WORKING_BALL_SENSOR(); /**< Mapping of robots with working ball sensors */
    static std::map<int, bool> ROBOTS_WITH_WORKING_DRIBBLER_ENCODER(); /**< Mapping of robots with working dribbler encoders */
    static std::map<int, bool> ROBOTS_WITH_KICKER(); /**< Mapping of robots with working kicker */
    static std::map<int, float>  ROBOTS_MAXIMUM_KICK_TIME(); /**< Mapping robots with their respective time to charge the kicker */

    static bool ROBOT_HAS_WORKING_DRIBBLER(int id); /**< Checks whether the given robot has a working dribbler */
    static bool ROBOT_HAS_WORKING_BALL_SENSOR(int id); /**< Checks whether the given robot has a working ball sensor */
    static bool ROBOT_HAS_WORKING_DRIBBLER_ENCODER(int id); /**< Checks whether the given robot has a working dribbler encoder */
    static bool ROBOT_HAS_KICKER(int id); /**< Checks whether the given robot has a working kicker */
    static int ROBOT_MAXIMUM_KICK_TIME(int id); /**< Checks the time the given robot need to charge the kicker */

    // Default PID values for the gotoposses/interface
    static pidVals standardNumTreePID(); /**< The standard PID values for NumTree */
    static pidVals standardReceivePID(); /**< The standard PID values for Receive */
    static pidVals standardInterceptPID(); /**< The standard PID values for Intercept */
    static pidVals standardKeeperPID(); /**< The standard PID values for Keeper*/
    static pidVals standardKeeperInterceptPID(); /**< The standard PID values for KeeperIntercept */

   private:
    static bool isInitialized; /**< Keeps track of whether the AI is initialized */
    static bool robotOutputTargetGrSim; /**< Keeps track of whether the output should go to GrSim */
};

}  // namespace rtt::ai
 /**
  * @brief Enumerators for the different game state given by the referee
  */
enum class RefCommand {
    /// OFFICIAL REFEREE GAME STATES ///
    HALT = 0,
    STOP = 1,
    NORMAL_START = 2,
    FORCED_START = 3,
    PREPARE_KICKOFF_US = 4,
    PREPARE_KICKOFF_THEM = 5,
    PREPARE_PENALTY_US = 6,
    PREPARE_PENALTY_THEM = 7,
    DIRECT_FREE_US = 8,
    DIRECT_FREE_THEM = 9,
    INDIRECT_FREE_US = 10,
    INDIRECT_FREE_THEM = 11,
    TIMEOUT_US = 12,
    TIMEOUT_THEM = 13,
    GOAL_US = 14,
    GOAL_THEM = 15,
    BALL_PLACEMENT_US = 16,
    BALL_PLACEMENT_THEM = 17,
    PRE_HALF = 26,

    /// CUSTOM GAME STATES ///
    DO_KICKOFF = 18,
    DEFEND_KICKOFF = 19,
    DO_PENALTY = 20,
    DEFEND_PENALTY = 21,
    PREPARE_SHOOTOUT_US = 22,
    PREPARE_SHOOTOUT_THEM = 23,
    DO_SHOOTOUT = 24,
    DEFEND_SHOOTOUT = 25,

    UNDEFINED = -1
};

#endif  // ROBOTEAM_AI_CONSTANTS_H

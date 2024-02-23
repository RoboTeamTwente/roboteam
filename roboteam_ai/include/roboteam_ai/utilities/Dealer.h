#ifndef RTT_ROBOTEAM_AI_SRC_UTILITIES_DEALER_H_
#define RTT_ROBOTEAM_AI_SRC_UTILITIES_DEALER_H_
/**
 * @file Dealer.hpp
 * @brief
 * @section Dealer
 * The purpose of Dealer is to go from pairs of (name -> dealerFlags) to pairs of (name -> robot id).
 * In other words, there is a requirement for each role in the tree, and Dealer maps the robot that fits those
 * requirements best to that role in the tree. The lower the score, the better.
 */
#include <iostream>
#include <map>
#include <roboteam_utils/Field.hpp>
#include <vector>

#include "gtest/gtest_prod.h"
#include "stp/StpInfo.h"
#include "world/views/RobotView.hpp"
#include "world/views/WorldDataView.hpp"

namespace rtt::ai {
namespace v = rtt::world::view;
/**
 * @brief Enumerator that defines the name of the dealerFlags
 */
enum class DealerFlagTitle { WITH_WORKING_BALL_SENSOR, WITH_WORKING_DRIBBLER, KEEPER, CAN_DETECT_BALL, CAN_KICK_BALL };

/**
 * @brief Enumerator that defines the priority of the dealerFlags
 */
enum class DealerFlagPriority { CARD, LOW_PRIORITY, MEDIUM_PRIORITY, HIGH_PRIORITY, REQUIRED, KEEPER };

/**
 * @brief The order at which the Priority will be dealt with (Keeper first, Low last).
 */
static std::vector<DealerFlagPriority> PriorityOrder{DealerFlagPriority::KEEPER,          DealerFlagPriority::REQUIRED,     DealerFlagPriority::HIGH_PRIORITY,
                                                     DealerFlagPriority::MEDIUM_PRIORITY, DealerFlagPriority::LOW_PRIORITY, DealerFlagPriority::CARD};

/**
 * @brief Class that defines the dealer. The dealer will assign a role to each robot according to their states
 */
class Dealer {
    FRIEND_TEST(DealerTest, it_properly_distributes_robots);
    FRIEND_TEST(DealerTest, the_score_factor_increases_with_priority);

   public:
    /**
     * @brief Structure that defines the dealer flag. The dealer flag will give a certain factor with the influence it should have on assigning the roles
     */
    struct DealerFlag {
        DealerFlagTitle title; /**< Factor that the dealer should take into account */
        /**
         * @brief Explicit constructor of the DealerFlag structure
         * @param title Title of the dealer flag
         */
        explicit DealerFlag(DealerFlagTitle title);
    };

    /// The priority of the role with the falgs that need to be considered.
    // Forced ID should be ONLY be used in situations where it would bypass the inefficiency of the world to ai communication
    // i.e. ball position and velocity when passing another robot in the previous play.
    /**
     * @brief Structure that defines the role info. The role info gives information about the role that should be assigned
     */
    struct RoleInfo {
        DealerFlagPriority priority;   /**< Priority of the role */
        std::vector<DealerFlag> flags; /**< Vector of dealer flags about the role */
        int forcedID = -1;             /**< The ID this role should be force pushed on, -1 if the dealer should decide */
    };

    using FlagMap = std::map<std::string, RoleInfo>; /**< Mapping of a role to its distribution information */
    /**
     * @brief Constructor of the dealer class
     * @param world The current world
     * @param field The current field
     */
    Dealer(v::WorldDataView world, rtt::Field *field);

    /**
     * @brief Virtual default destructor of the dealer class
     */
    virtual ~Dealer() = default;

    /**
     * @brief Distributes the role between the friendly robots considering all information given in the role_to_flags.
     * Robots and role_to_flags are copies as they are modified to reduce computations after the forcedID's.
     * @param robots Vector containing all friendly robots
     * @param role_to_flags Information map with all information for each role (with key -> roleName)
     * @param stpInfoMap Mapping of stpInfos to their respective role
     * @return Mapping of the roleName and the Robot that should get the role
     */
    std::unordered_map<std::string, v::RobotView> distribute(std::vector<v::RobotView> robots, FlagMap role_to_flags,
                                                             const std::unordered_map<std::string, stp::StpInfo> &stpInfoMap);

   protected:
    /**
     * @brief Getter for the default flag scores
     * This function is virtual such that it can be mocked in the tests.
     * the performance hit is minimal (in the scope of nanoseconds)
     * @param robot Robot for which the flag should be scored
     * @param flag The flag that needs to be scored
     * @return Score of the given flag
     */
    virtual double getDefaultFlagScores(const v::RobotView &robot, const DealerFlag &flag);

   private:
    v::WorldDataView world; /**< The world data */
    rtt::Field *field;      /**< The field data */

    /**
     * @brief Score of a certain flag (/factor) with its weight that it was multiplied with (to be used in normalization)
     */
    struct FlagScore {
        double score;  /**< Score of the flag */
        double weight; /**< Weight of the score */
    };

    /**
     * @brief Calculates the score for a distance between a point and a robot
     * @param stpInfo The information that is needed for calculating the score
     * @param robot The robot for which the distance needs to be scored
     * @return Score for distance
     */
    double getRobotScoreForDistance(const stp::StpInfo &stpInfo, const v::RobotView &robot);

    /**
     * @brief Populates the matrix of robots and roles with scores based on flags and distance
     * @param allRobots All robots that need to be assigned a role
     * @param flagMap The flags that should be taken into account for each robot
     * @param stpInfoMap Mapping of stpInfos to each role
     * @return The score matrix
     */
    std::vector<std::vector<double>> getScoreMatrix(const std::vector<v::RobotView> &allRobots, const FlagMap &flagMap,
                                                    const std::unordered_map<std::string, stp::StpInfo> &stpInfoMap);

    /**
     * @brief Translates a priority into a double
     * @param flagPriority The priority that needs to become a weight
     * @return Priority weight
     */
    static double getWeightForPriority(const DealerFlagPriority &flagPriority);

    /**
     * @brief Calculates the cost of travelling a certain distance
     * @param distance  The distance to travel
     * @param fieldHeight The height of the field
     * @return Cost of travelling that distance
     */
    static double costForDistance(double distance, double fieldHeight);

    /**
     * @brief Calculates the cost of a property of a robot
     * @param property The property of a robot, for example a working dribbler
     * @return Cost of a property
     */
    static double costForProperty(bool property);

    /**
     * @brief Calculates the score for all flags for a role, for one robot (so will be called 11 times for each role)
     * @param dealerFlags The flags that need to be scored
     * @param robot The robot that the score should be calculated for
     * @return The score of all flags combined
     */
    double getRobotScoreForRole(const std::vector<Dealer::DealerFlag> &dealerFlags, const v::RobotView &robot);

    /**
     * @brief Distributes the forced roles first, so that other rest of the function does not need to compute extra information
     * @param robots A copy of all our robots, where in the robots will be removed
     * @param flagMap Where in the roles will be removed
     * @param assignments Mapping of each role to a robot
     */
    void distributeFixedIds(std::vector<v::RobotView> &robots, FlagMap &flagMap, std::unordered_map<std::string, v::RobotView> &assignments);

    /**
     * @brief Sets the keeper and ballplacer id in the gamestate if either of those roles are distributed by the dealer
     * @param output The role division to be distributed
     */
    void setGameStateRoleIds(std::unordered_map<std::string, v::RobotView> output);
};
}  // namespace rtt::ai
#endif  // RTT_ROBOTEAM_AI_SRC_UTILITIES_DEALER_H_
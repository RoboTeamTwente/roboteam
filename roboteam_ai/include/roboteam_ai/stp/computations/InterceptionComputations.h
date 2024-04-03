#ifndef RTT_INTERCEPTIONCOMPUTATIONS_H
#define RTT_INTERCEPTIONCOMPUTATIONS_H

#include <roboteam_utils/Arc.h>
#include <roboteam_utils/Circle.h>
#include <roboteam_utils/Grid.h>
#include <roboteam_utils/Line.h>

#include <cmath>
#include <optional>
#include <roboteam_utils/Field.hpp>

#include "stp/Role.hpp"
#include "stp/StpInfo.h"
#include "stp/constants/GeneralizationConstants.h"
#include "utilities/Constants.h"
#include "world/FieldComputations.h"
#include "world/World.hpp"
#include "world/views/WorldDataView.hpp"

namespace rtt::ai::stp {

struct InterceptionInfo {
    Vector2 interceptLocation;
    int interceptId = -1;
    double timeToIntercept = std::numeric_limits<double>::max();
};

struct KeeperInterceptionInfo {
    bool canIntercept = false;
    Vector2 interceptLocation;
};

/**
 * @brief class with computations about positions
 */
class InterceptionComputations {
   public:
    /**
     * @brief Calculates keeper intercept info, whether he will intercept and where to do so. If the ball will be inside the defense area in the next second, the keeper will
     * intercept the ball if he can reach the ball in that time and no enemy is near.
     * @param world The current world
     * @param keeper The keeper
     * @return struct with information about the keeper intercept
     */
    static KeeperInterceptionInfo calculateKeeperInterceptionInfo(const world::World *world, const world::view::RobotView &keeper) noexcept;

    /**
     * @brief Calculates the interception info for the passer, will return who can be at the ball first. If no one can reach the ball, will return who can be the closest to the
     * ball
     * @param ourRobots The robots of our team that we want to consider (usually only the robots that can kick or dribble the ball)
     * @param world The current world
     * @return struct with information about the interception
     */
    static InterceptionInfo calculateInterceptionInfo(const std::vector<world::view::RobotView> &ourRobots, const world::World *world);

    /**
     * @brief Determines which robot should be the keeper. This is either the robot which id is the keeperId in the GameState. If this keeperId is not in the field, we pick the
     * robot closest to our goal. This is to make sure that even when keeperId is set incorrectly, we still have the correct robot as keeper and don't assign him another
     * ball-interception role.
     * @param possibleRobots vector of robots that could become keeper
     * @param world current world
     * @return Id of robot that should become keeper and thus ignored for interception
     */
    static int getKeeperId(const std::vector<world::view::RobotView> &possibleRobots, const world::World *world);

    /**
     * @brief Determines which robot should be the passer (the robot closest to the ball)
     * @param possibleRobots vector of robots that could become passer
     * @param world current world
     * @return Id of robot that should become passer
     */
    static InterceptionInfo calculateInterceptionInfoForKickingRobots(const std::vector<world::view::RobotView> &possibleRobots, const world::World *world);

    /**
     * @brief Calculates the interceptionInfo using the calculateInterceptionInfo function, considering robots that can not kick or are the keeper
     * @param world The current world
     * @return HarasserInfo with the id and the time to the ball
     */
    static InterceptionInfo calculateInterceptionInfoExcludingKeeperAndCarded(const rtt::world::World *world) noexcept;
};
}  // namespace rtt::ai::stp
#endif  // RTT_INTERCEPTIONCOMPUTATIONS_H

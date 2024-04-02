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
#include "stp/computations/PassComputations.h"
#include "stp/constants/GeneralizationConstants.h"
#include "utilities/Constants.h"
#include "world/FieldComputations.h"
#include "world/World.hpp"
#include "world/views/WorldDataView.hpp"

namespace rtt::ai::stp {

struct InterceptInfo {
    Vector2 interceptLocation;
    int interceptId = -1;
    double timeToIntercept = std::numeric_limits<double>::max();
};

struct KeeperInterceptInfo {
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
    static KeeperInterceptInfo calculateKeeperInterceptionInfo(const world::World *world, const world::view::RobotView &keeper) noexcept;

    /**
     * @brief Calculates the interception info for the passer, will return who can be at the ball first. If no one can reach the ball, will return who can be the closest to the
     * ball
     * @param ourRobots The robots of our team that we want to consider (usually only the robots that can kick or dribble the ball)
     * @param world The current world
     * @return struct with information about the interception
     */
    static InterceptInfo calculateInterceptionInfo(const std::vector<world::view::RobotView> &ourRobots, const world::World *world);

    /**
     * @brief Calculates the id of the harasser using the calculateInterceptionInfo function
     * @param world The current world
     * @return HarasserInfo with the id and the time to the ball
     */
    static InterceptInfo calculateGetBallId(const rtt::world::World *world) noexcept;
};
}  // namespace rtt::ai::stp
#endif  // RTT_INTERCEPTIONCOMPUTATIONS_H

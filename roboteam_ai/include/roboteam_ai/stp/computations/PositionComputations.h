#ifndef RTT_POSITIONCOMPUTATIONS_H
#define RTT_POSITIONCOMPUTATIONS_H

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

struct EnemyInfo {
    Vector2 position;
    Vector2 velocity;
    int id;
};

/**
 * @brief class with computations about positions
 */
class PositionComputations {
   public:
    /**
     * @brief Determines the location for defenders around the defense area
     * Uses the defence area boundaries and the path from ball to center of goal to find the intersects of circles to
     * find the various positions.
     * @param field
     * @param world
     * @param amountDefenders to be placed
     * @return vector with Vector2 positions for each of the defenders
     */
    static std::vector<Vector2> determineWallPositions(const Field &field, const world::World *world, int amountDefenders);

    /**
     * @brief Returns the best scored position from a grid with a profile
     * @param currentPosition The position the robot it currently going to (small biased) if it exists
     * @param searchGrid the area (with points) that should be searched
     * @param profile combination of weights for different factors that should be scored
     * @param field
     * @param world
     * @return the best position within that grid with its score
     */
    static gen::ScoredPosition getPosition(std::optional<rtt::Vector2> currentPosition, const Grid &searchGrid, gen::ScoreProfile profile, const Field &field,
                                           const world::World *world);

    /**
     * @brief Makes a wall if not ready done, saves it in calculatedWallPositions and deals the index
     * @param index Index of the wall position (do unique positions)
     * @param amountDefenders Amount of defenders the wall is made of
     * @param field
     * @param world
     * @return Vector2 position of that index in the wall
     */
    static Vector2 getWallPosition(int index, int amountDefenders, const Field &field, world::World *world);

    /**
     * @brief Calculates the position of the robot to avoid other robots
     * @param targetPosition The initial target position
     * @param world The current world
     * @param robotId The id of the robot
     * @param avoidObj The objects to avoid
     * @param field The current field
     * @return A position that is not too close to other robots
     */
    static Vector2 calculateAvoidRobotsPosition(Vector2 targetPosition, const world::World *world, int robotId, const AvoidObjects &avoidObj, const Field &field);
    /**
     * @brief Calculates a position, near the target position, that is not too close to the ball
     * @param targetPosition The initial target position
     * @param ballPosition The position of the ball
     * @param field The current field
     * @return A position that is not within the min allowed distance to the ball
     */
    static Vector2 calculateAvoidBallPosition(Vector2 targetPosition, Vector2 ballPosition, const Field &field);

    /**
     * @brief Calculates info for the harasser role
     * @param stpInfos The current stpInfos
     * @param roles The current roles
     * @param field The current field
     * @param world The current world
     * @param interceptionLocation The location where the harasser should go to
     */
    static void calculateInfoForHarasser(std::unordered_map<std::string, StpInfo> &stpInfos, std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT> *roles,
                                         const Field &field, world::World *world, Vector2 interceptionLocation) noexcept;

    /**
     * @brief Calculates info for the defenders
     * @param stpInfos The current stpInfos
     * @param roles The current roles
     * @param field The current field
     * @param world The current world
     * @param mustStayOnOurSide Whether the defenders should always stay on our side of the field, for example to prevent interference during our own attack
     */
    static void calculateInfoForDefendersAndWallers(std::unordered_map<std::string, StpInfo> &stpInfos,
                                                    std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT> &roles, const Field &field, world::World *world,
                                                    bool mustStayOnOurSide) noexcept;

    /**
     * @brief Calculates info for the attackers
     * @param stpInfos The current stpInfos
     * @param roles The current roles
     * @param field The current field
     * @param world The current world
     */
    static void calculateInfoForAttackers(std::unordered_map<std::string, StpInfo> &stpInfos, std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT> &roles,
                                          const Field &field, world::World *world) noexcept;

    /**
     * @brief Calculates info for the formations
     * @param stpInfos The current stpInfos
     * @param roles The current roles
     * @param field The current field
     * @param world The current world
     */
    static void calculateInfoForFormation(std::unordered_map<std::string, StpInfo> &stpInfos, std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT> &roles,
                                          const Field &field, world::World *world) noexcept;

    /**
     * @brief Calculates info for the formations on our side of the field
     * @param stpInfos The current stpInfos
     * @param roles The current roles
     * @param field The current field
     * @param world The current world
     */
    static void calculateInfoForFormationOurSide(std::unordered_map<std::string, StpInfo> &stpInfos,
                                                 std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT> &roles, const Field &field,
                                                 world::World *world) noexcept;
    /**
     * @brief Recalculates info for the position of our robots to not interfere with passing
     * @param stpInfos The current stpInfos
     * @param field The current field
     * @param world The current world
     * @param passInfo The current passInfo
     */
    static void recalculateInfoForNonPassers(std::unordered_map<std::string, StpInfo> &stpInfos, const Field &field, world::World *world, Vector2 passLocation) noexcept;

   private:
    /**
     * @brief Calculates a position outside of a given shape
     * @param targetPosition The position where the robot wants to go
     * @param field The current field
     * @param avoidShape The shape to avoid
     * @return A position that is outside the given shape
     */
    static Vector2 calculatePositionOutsideOfShape(Vector2 targetPosition, const Field &field, const std::unique_ptr<Shape> &avoidShape);
};
}  // namespace rtt::ai::stp
#endif  // RTT_POSITIONCOMPUTATIONS_H

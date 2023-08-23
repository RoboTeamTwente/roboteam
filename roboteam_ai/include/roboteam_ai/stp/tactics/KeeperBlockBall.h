//
// Created by jordi on 08-04-20.
//

#ifndef RTT_KEEPERBLOCKBALL_H
#define RTT_KEEPERBLOCKBALL_H

#include <roboteam_utils/HalfLine.h>

#include "stp/Tactic.h"

namespace rtt::ai::stp::tactic {
/**
 * @brief Class that defines the keeper block ball tactic. This is used when the keeper should focus on blocking the ball from going into the goal
 */
class KeeperBlockBall : public Tactic {
   public:
    /**
     * @brief Constructor for the tactic, it constructs the state machine of skills
     */
    KeeperBlockBall();

   private:
    /**
     * @brief Calculate the info for skills from the StpInfo struct parameter
     * @param info info is the StpInfo passed by the role
     * @return std::optional<SkillInfo> based on the StpInfo parameter
     */
    std::optional<StpInfo> calculateInfoForSkill(StpInfo const &info) noexcept override;

    /**
     * @brief Is this tactic failing during execution (go back to the previous tactic)
     * This tactic can never fail, so always returns false
     * @param info StpInfo can be used to check some data
     * @return true, tactic will fail (go back to prev tactic), false execution will continue as usual
     */
    bool isTacticFailing(const StpInfo &info) noexcept override;

    /**
     * @brief Should this tactic be reset (go back to the first skill of this tactic)
     * Returns true when the robot is further away from the target position than some error margin
     * @param info StpInfo can be used to check some data
     * @return true if tactic  should reset, false if execution should continue
     */
    bool shouldTacticReset(const StpInfo &info) noexcept override;

    /**
     * @brief Checks whether this is a passive tactic (formerly called endTactic)
     * @return This will always return true, since it is an endTactic
     */
    bool isEndTactic() noexcept override;

    /**
     * @brief Gets the tactic name
     * @return The name of this tactic
     */
    const char *getName() override;

    /**
     * @brief Creates a LineSegment on which the keeper should stay while defending.
     * Start is left of goal, end is right of goal
     * @return the keepers lineSegment
     */
    static LineSegment getKeepersLineSegment(const Field &);

    /**
     * @brief Estimates the trajectory of the ball, either from current velocity or from enemies that might kick it
     * @param ball to estimate its trajectory from
     * @param enemyRobot an enemy that might manipulate the ball
     * @return a trajectory the ball might take
     */
    static std::optional<HalfLine> estimateBallTrajectory(const world::view::BallView &ball, const std::optional<world::view::RobotView> &enemyRobot);

    /**
     * @brief Checks if the given trajectory goes towards our goal
     * @param ballTrajectory the trajectory of the ball
     * @param field which contains our goal
     * @return true if the trajectory goes near our goal, false otherwise
     */
    static bool isBallHeadingTowardsOurGoal(const HalfLine &ballTrajectory, const Field &field);

    /**
     * @brief Calculates the position for the keeper and the PID type with which that position should be achieved
     * PID type is different for intercepting and kicking (coarse and fast or fine and slower control)
     * @param ball is the current ball for which the keeper should defend
     * @param field is the field on which the keeper should defend
     * @param enemyRobot Enemy robot closest to ball
     * @return Target position for the keeper and the corresponding PID type
     */
    static std::pair<Vector2, PIDType> calculateTargetPosition(const world::view::BallView &ball, const Field &field,
                                                               const std::optional<world::view::RobotView> &enemyRobot) noexcept;

    /**
     * @brief Calculates the angle the robot should have
     * @param ball the ball for which the keeper should defend
     * @param targetKeeperPosition the target position the keeper should have
     * @return the angle the robot should have
     */
    static Angle calculateTargetAngle(const world::view::BallView &ball, const Vector2 &targetKeeperPosition);
};
}  // namespace rtt::ai::stp::tactic

#endif  // RTT_KEEPERBLOCKBALL_H

//
// Created by jessevw on 12.03.20.
//

#ifndef RTT_BLOCKROBOT_H
#define RTT_BLOCKROBOT_H

#include "stp/Tactic.h"

namespace rtt::ai::stp::tactic {
/**
 * @brief Class that defines the block robot tactic. This tactic is used for blocking a robot from going to a position
 */
class BlockRobot : public Tactic {
   public:
    /**
     * @brief Constructor for the tactic, it constructs the state machine of skills
     */
    BlockRobot();

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
     * @return true if tactic should reset, false if execution should continue
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
     * @brief Find the desired angle for the robot to block the target
     * @param enemy the robot that is being blocked
     * @param targetLocation the location that you want to block off from the robot. For example, the ball position or our goal
     * @return desired angle for robot to block target
     */
    double calculateAngle(const world::view::RobotView enemy, const Vector2 &targetLocation);

    /**
     * @brief Find location for robot to move to to block the target
     * @param blockDistance how close the robot should be to the enemy robot.
     * @param enemy the enemy robot to be blocked
     * @param targetLocation the location that you want to block off from the robot. For example, the ball position or our goal
     * @return the desired position to block the target.
     */
    Vector2 calculateDesiredRobotPosition(BlockDistance blockDistance, const world::view::RobotView enemy, const Vector2 &targetLocation, double enemyDistance);
};
}  // namespace rtt::ai::stp::tactic

#endif  // RTT_BLOCKROBOT_H

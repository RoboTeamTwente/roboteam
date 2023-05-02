//
// Created by jordi on 13-03-20.
//

#ifndef RTT_RECEIVE_H
#define RTT_RECEIVE_H

#include "stp/Tactic.h"

namespace rtt::ai::stp::tactic {
/**
 * @brief Class that defines the receive tactic. This tactic is used when the ball has been passed to this robot and it should receive the ball
 */
class Receive : public Tactic {
   public:
    /**
     * @brief Constructor for the tactic, it constructs the state machine of skills
     */
    Receive();

   private:
    /**
     * @brief Calculate the info for skills from the StpInfo struct parameter
     * @param info info is the StpInfo passed by the role
     * @return std::optional<SkillInfo> based on the StpInfo parameter
     */
    std::optional<StpInfo> calculateInfoForSkill(StpInfo const &info) noexcept override;

    /**
     * @brief Is this tactic failing during execution (go back to the previous tactic)
     * This tactic fails if there is no move to position
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
     * @brief Checks whether this is a passive tactic (formerly called endTactic
     * @return This will always return true, since it is an endTactic
     */
    bool isEndTactic() noexcept override;

    /**
     * @brief Gets the tactic name
     * @return The name of this tactic
     */
    const char *getName() override;

    /**
     * @brief Calculates the angle the robot needs to have using the ball position
     * @param robot
     * @param ball
     * @return Angle the robot should take
     */
    double calculateAngle(const world::view::RobotView &robot, const world::view::BallView &ball);
};
}  // namespace rtt::ai::stp::tactic

#endif  // RTT_RECEIVE_H

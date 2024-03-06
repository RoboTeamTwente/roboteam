//
// Created by jordi on 06-04-20.
//

#ifndef RTT_GETBEHINDBALLINDIRECTION_H
#define RTT_GETBEHINDBALLINDIRECTION_H

#include "stp/Tactic.h"

namespace rtt::ai::stp::tactic {
/**
 * @brief Class that defines the get behind ball in direction tactic. This tactic is used for getting the ball from a certain direction.
 * It has 3 skills: GoToPos, Rotate, and GoToPos.
 * It fails when there is no target to point towards and does not reset.
 * It's not an end tactic, therefore it can succeed.
 */
class GetBehindBallInDirection : public Tactic {
   public:
    /**
     * @brief Constructor for the tactic, it constructs the state machine of skills
     */
    GetBehindBallInDirection();

   private:
    /**
     * @brief Calculate the info for skills from the StpInfo struct parameter
     * @param info info is the StpInfo passed by the role
     * @return std::optional<SkillInfo> based on the StpInfo parameter
     */
    std::optional<StpInfo> calculateInfoForSkill(StpInfo const &info) noexcept override;

    /**
     * @brief Calculates the position to go to for GoToPos
     * @param ballPosition The position of the ball
     * @param robotPosition The position of the robot
     * @param positionToShootAt The position to shoot at
     * @return The position to move to during GoToPos
     */
    Vector2 calculateTargetPosition(Vector2 ballPosition, Vector2 robotPosition, Vector2 positionToShootAt);

    /**
     * @brief Is this tactic failing during execution (go back to the previous tactic)
     * Fails when there is no target to shoot at
     * @param info StpInfo can be used to check some data
     * @return true, tactic will fail (go back to prev tactic), false execution will continue as usual
     */
    bool isTacticFailing(const StpInfo &info) noexcept override;

    /**
     * @brief Should this tactic be reset (go back to the first skill of this tactic)
     * This tactic never resets, so false
     * @param info StpInfo can be used to check some data
     * @return true if tactic  should reset, false if execution should continue
     */
    bool shouldTacticReset(const StpInfo &info) noexcept override;

    /**
     * @brief Returns whether this is an end tactic, meaning it should keep looping this tactic if all skills are finished
     * @return This will always return false, since it is NOT an endTactic
     */
    bool isEndTactic() noexcept override;

    /**
     * @brief Gets the tactic name
     * @return The name of this tactic
     */
    const char *getName() override;
};
}  // namespace rtt::ai::stp::tactic

#endif  // RTT_GETBEHINDBALLINDIRECTION_H

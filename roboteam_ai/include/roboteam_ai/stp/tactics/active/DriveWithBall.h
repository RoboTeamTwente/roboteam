//
// Created by timovdk on 3/16/20.
//

#ifndef RTT_DRIVEWITHBALL_H
#define RTT_DRIVEWITHBALL_H

#include <stp/Tactic.h>

namespace rtt::ai::stp::tactic {
class DriveWithBall : public Tactic {
   public:
    /**
     * Constructor for the tactic, it constructs the state machine of skills
     */
    DriveWithBall();

   private:
    /**
     * Calculate the info for skills from the StpInfo struct parameter
     * @param info info is the StpInfo passed by the role
     * @return std::optional<SkillInfo> based on the StpInfo parameter
     */
    std::optional<StpInfo> calculateInfoForSkill(StpInfo const &info) noexcept override;

    /**
     * Is this tactic failing during execution (go back to the previous tactic)
     * @param info StpInfo can be used to check some data
     * @return true, tactic will fail (go back to prev tactic), false execution will continue as usual
     * Fails if we don't have the ball or there is no movement position
     */
    bool isTacticFailing(const StpInfo &info) noexcept override;

    /**
     * Should this tactic be reset (go back to the first skill of this tactic)
     * @param info StpInfo can be used to check some data
     * @return true if tactic  should reset, false if execution should continue
     * Should reset if the angle the robot is at is no longer correct
     */
    bool shouldTacticReset(const StpInfo &info) noexcept override;

    /**
     * Checks if this tactic should be forced success
     * It is forced success whenever we are at the right location
     * @param info
     * @return whether the tactic is forced success
     */
    bool forceTacticSuccess(const StpInfo &info) noexcept override;

    /**
     * Is this tactic an end tactic?
     * @return This will always return false, since it is NOT an endTactic
     */
    bool isEndTactic() noexcept override;

    /**
     * Gets the tactic name
     * @return The name of this tactic
     */
    const char *getName() override;
};
}  // namespace rtt::ai::stp::tactic

#endif  // RTT_DRIVEWITHBALL_H

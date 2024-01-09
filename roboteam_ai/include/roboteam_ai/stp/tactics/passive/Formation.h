//
// Created by timovdk on 3/27/20.
//

#ifndef RTT_FORMATION_TACTIC_H
#define RTT_FORMATION_TACTIC_H

#include "stp/Tactic.h"

namespace rtt::ai::stp::tactic {
/**
 * @brief Class that defines the formation tactic. This tactic is used when the robot just needs to be on standby in a certain position
 */
class Formation : public Tactic {
   public:
    /**
     * @brief Constructor for the tactic, it constructs the state machine of skills
     */
    Formation();

   private:
    /**
     * @brief Calculate the info for skills from the StpInfo struct parameter
     * @param info info is the StpInfo passed by the role
     * @return std::optional<SkillInfo> based on the StpInfo parameter
     */
    std::optional<StpInfo> calculateInfoForSkill(StpInfo const &info) noexcept override;

    /**
     * @brief Is this tactic failing during execution (go back to the previous tactic)
     * Fails if there is no position to move to
     * @param info StpInfo can be used to check some data
     * @return true, tactic will fail (go back to prev tactic), false execution will continue as usual
     */
    bool isTacticFailing(const StpInfo &info) noexcept override;

    /**
     * @brief Should this tactic be reset (go back to the first skill of this tactic)
     * Resets when robot position is not close enough to the target position
     * @param info StpInfo can be used to check some data
     * @return true if tactic  should reset, false if execution should continue
     */
    bool shouldTacticReset(const StpInfo &info) noexcept override;

    /**
     * @brief Checks whether this is an end tactic, meaning it should keep looping this tactic if all skills are finished
     * @return This will always return true, since it is an endTactic
     */
    bool isEndTactic() noexcept override;

    /**
     * @brief Gets the tactic name
     * @return The name of this tactic
     */
    const char *getName() override;
};
}  // namespace rtt::ai::stp::tactic

#endif  // RTT_FORMATION_TACTIC_H

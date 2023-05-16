//
// Created by timovdk on 3/13/20.
// rotate and chip

#ifndef RTT_CHIPATPOS_H
#define RTT_CHIPATPOS_H

#include "stp/Tactic.h"

namespace rtt::ai::stp::tactic {
/**
 * @brief Class that defines the ChipAtPos tactic. The robot will chip the ball to a given position
 */
class ChipAtPos : public Tactic {
   public:
    /**
     * @brief Constructor for the tactic, it constructs the state machine of skills
     */
    ChipAtPos();

   private:
    /**
     * @brief Calculate the info for skills from the StpInfo struct parameter
     * @param info info is the StpInfo passed by the role
     * @return std::optional<SkillInfo> based on the StpInfo parameter
     */
    std::optional<StpInfo> calculateInfoForSkill(StpInfo const &info) noexcept override;

    /**
     * @brief Is this tactic failing during execution (go back to the previous tactic)
     * @param info StpInfo can be used to check some data
     * @return true, tactic will fail (go back to prev tactic), false execution will continue as usual
     * Returns true when robot doesn't have the ball or if there is no shootTarget
     */
    bool isTacticFailing(const StpInfo &info) noexcept override;

    /**
     * @brief Should this tactic be reset (go back to the first skill of this tactic)
     * @param info StpInfo can be used to check some data
     * @return true if tactic  should reset, false if execution should continue
     * Returns true when the robot angle is not within its error margin
     */
    bool shouldTacticReset(const StpInfo &info) noexcept override;

    /**
     * @brief Is this tactic an end tactic?
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

#endif  // RTT_CHIPATPOS_H

//
// Created by Luuk and Jorn on 14/09/2023.
//

#ifndef RTT_DEMOKEEPERDEFENDGOAL_H
#define RTT_DEMOKEEPERDEFENDGOAL_H

#include "stp/Skill.h"

namespace rtt::ai::stp::skill {
/**
 * @brief Class that defines the go to position skill. This skill is used when the robot should move to a given position
 */
class DemoKeeperDefendGoal : public Skill {
    /**
     * @Brief On update of this tactic
     * @param info StpInfo struct with all relevant info for this robot and this skill
     * @return A Status, either Running or Success
     */
    Status onUpdate(StpInfo const& info) noexcept override;

    /**
     * @brief Gets the skill name
     * @return The name of this skill
     */
    const char* getName() override;
};
}  // namespace rtt::ai::stp::skill

#endif  // RTT_DEMOKEEPERDEFENDGOAL_H

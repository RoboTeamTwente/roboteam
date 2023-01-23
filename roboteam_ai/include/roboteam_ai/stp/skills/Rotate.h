//
// Created by jordi on 06-03-20.
//

#ifndef RTT_ROTATE_H
#define RTT_ROTATE_H

#include "stp/Skill.h"

namespace rtt::ai::stp::skill {
/**
 * @brief Class that defines the rotate skill. This is a very slow way of turning with the ball but is also very safe as we won't crash into things.
 * This skill is useful when we don't have to be fast but we want to ensure that we can turn with the ball safely (for example during ball placement)
 */
class Rotate : public Skill {
    /**
     * @brief On update of this tactic
     * @param info StpInfo struct with all relevant info for this robot and this skill
     * @return A Status, either Running or Success
     */
    Status onUpdate(StpInfo const& info) noexcept override;

    /**
     * @brief Gets the skill name
     * @return The name of this skill
     */
    const char* getName() override;

    int withinMarginCount = 0; /**< Counts how long the robot is within the rotational error margin */
};
}  // namespace rtt::ai::stp::skill

#endif  // RTT_ROTATE_H

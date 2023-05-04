//
// Created by rtt-vision on 12-10-21.
//

#ifndef RTT_TESTSKILL_H
#define RTT_TESTSKILL_H

#include "stp/Skill.h"

namespace rtt::ai::stp::skill {
/**
 * @brief Class that defines the test skill. This skill is used for testing
 */
class TestSkill : public Skill {
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
};
}  // namespace rtt::ai::stp::skill
#endif  // RTT_TESTSKILL_H

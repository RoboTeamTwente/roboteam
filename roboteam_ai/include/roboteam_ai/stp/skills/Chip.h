#ifndef RTT_CHIP_H
#define RTT_CHIP_H

#include "stp/Skill.h"

namespace rtt::ai::stp::skill {
/**
 * @brief Class that defines the chipping skill. The robot will chip the ball a given distance
 */
class Chip : public Skill {
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

#endif  // RTT_CHIP_H
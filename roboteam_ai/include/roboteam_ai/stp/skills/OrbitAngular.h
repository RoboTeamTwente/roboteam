#ifndef RTT_ORBITANGULAR_H
#define RTT_ORBITANGULAR_H

#include "stp/Skill.h"

namespace rtt::ai::stp::skill {
/**
 * @brief Class that defines the orbit angular skill. This is a more advanced version of the orbit skill,
 * the biggest difference being that the robot uses both angular and absolute velocities
 */
class OrbitAngular : public Skill {
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

    int withinMarginCount = 0; /**< Counts how long the robot is within the yaw error margin */
};
}  // namespace rtt::ai::stp::skill

#endif  // RTT_ORBITANGULAR_H

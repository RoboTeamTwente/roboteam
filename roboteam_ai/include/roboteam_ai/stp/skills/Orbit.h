#ifndef RTT_ORBIT_H
#define RTT_ORBIT_H

#include "roboteam_utils/pid.h"
#include "stp/Skill.h"

namespace rtt::ai::stp::skill {
/**
 * @brief Class that defines the orbit skill. This skill is used when the robot should orbit around the ball, this is a more advanced way of turning with the ball
 */
class Orbit : public Skill {
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

    int counter = 0; /**< Counter for how many ticks the robot is within the error margin */

    PID velPid = PID(0.75, 0, 0); /**< PID controller to determine velocity multiplier */
};
}  // namespace rtt::ai::stp::skill

#endif  // RTT_ORBIT_H

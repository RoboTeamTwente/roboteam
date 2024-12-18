#pragma once

#include "stp/Skill.h"

namespace rtt::ai::stp::skill {

class OrbitAngularAndDrive : public Skill {
   public:
    /**
     * @brief Executes the skill to move while rotating
     * @param info StpInfo struct with relevant robot & world info
     * @return Status indicating whether the skill is running or finished
     */
    Status onUpdate(const StpInfo& info) noexcept override;

    /**
     * @brief Gets the name of this skill
     * @return The name of this skill
     */
    const char* getName() override;

   private:
    // Counter to track how long robot has been within error margin
    int withinMarginCount{};
};

} // namespace rtt::ai::stp::skill
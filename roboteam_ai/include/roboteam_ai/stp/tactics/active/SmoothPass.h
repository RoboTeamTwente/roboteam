#pragma once

#include "stp/Tactic.h"

namespace rtt::ai::stp::tactic {

class SmoothPass : public Tactic {
   public:
    /**
     * @brief Constructor for the tactic, initializes skills state machine
     */
    SmoothPass();

    /**
     * @brief Calculates the info for the skills from the StpInfo struct
     * @param info info is the StpInfo struct that contains all relevant info for the tactic
     * @return std::optional<StpInfo> based on the success of calculations
     */
    std::optional<StpInfo> calculateInfoForSkill(const StpInfo& info) noexcept override;

    /**
     * @brief Is this tactic failing during execution (go back to previous tactic)
     * @param info StpInfo struct with relevant robot & world info
     * @return true if the tactic is failing, false otherwise
     */
    bool isTacticFailing(const StpInfo& info) noexcept override;

    /**
     * @brief Should this tactic be reset (go back to the first skill)
     * @param info StpInfo struct with relevant robot & world info
     * @return true if the tactic should reset, false otherwise
     */
    bool shouldTacticReset(const StpInfo& info) noexcept override;

    /**
     * @brief Is this tactic an end tactic?
     * @return true if this is an end tactic, false otherwise
     */
    bool isEndTactic() noexcept override;

    /**
     * @brief Gets the name of this tactic
     * @return name of this tactic
     */
    const char* getName() override;
};

}  // namespace rtt::ai::stp::tactic
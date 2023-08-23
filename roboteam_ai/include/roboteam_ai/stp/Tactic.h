//
// Created by jessevw on 03.03.20.
//

#ifndef RTT_TACTIC_H
#define RTT_TACTIC_H

#include <roboteam_utils/containers/state_machine.hpp>
#include <vector>

#include "stp/Skill.h"
#include "stp/StpInfo.h"

namespace rtt::ai::stp {
class Skill;
/**
 * @brief Base class of tactic
 */
class Tactic {
   protected:
    Status currentStatus{}; /**< Status of tactic, from last time update() was called */

    /**
     * @brief This function should calculate any extra information that the skills might need to be executed.
     * Things that should be calculated are for example how hard the kicker should shoot to get to the desired position
     * or how fast the dribbler should be spinning.
     * Though this method is responsible for ensuring everything is calculated, it helps to use helpers so this
     * function doesn't become a massive hack
     * @param info StpInfo struct that contains the info passed down from Role
     * @return an StpInfo struct with extra information for the skill
     */
    virtual std::optional<StpInfo> calculateInfoForSkill(StpInfo const &info) noexcept = 0;

    /**
     * @brief The condition when the current tactic fails
     * @param info the tactic info passed down from the play
     * @return true if the tactic's prerequisites are no longer met (e.g. losing the ball)
     */
    virtual bool isTacticFailing(const StpInfo &info) noexcept = 0;

    /**
     * @brief When the state should reset
     * @param info the tactic info passed down from the play
     * @return true if the active skill cannot execute (it's prerequisites are no longer met)
     */
    virtual bool shouldTacticReset(const StpInfo &info) noexcept = 0;

    /**
     * @brief Check if the current tactic is an end tactic - only Running or Failure status
     * @return true if the current tactic cannot succeed (i.e. is an end tactic)
     */
    virtual bool isEndTactic() noexcept = 0;

    rtt::collections::state_machine<Skill, Status, StpInfo> skills; /**< The state machine of skills */

   public:
    /**
     * @brief Gets the current status
     * @return The current status
     */
    [[nodiscard]] Status getStatus() const;

    /**
     * @brief Calls onInitialize of the tactic
     */
    void initialize() noexcept;

    /**
     * @brief Check if state machine is done, calls calculateInfoForSkill, calls update on the state machine with SkillInfo and calls onUpdate of this tactic for extra
     * customization
     * @param info info passed by the Role
     * @return Status of the skill that is currently being ticked
     */
    Status update(StpInfo const &info) noexcept;

    /**
     * @brief Calls onTerminate
     */
    void terminate() noexcept;

    /**
     * @brief Ensure proper destruction of Tactic classes
     */
    virtual ~Tactic() = default;

    /**
     * @brief Default constructor, ensures proper construction of Tactic
     */
    Tactic() = default;

    /**
     * @brief Default move-constructor, ensures proper move-construction of Tactic
     * @param other Tactic that should be moved
     */
    Tactic(Tactic &&other) = default;

    /**
     * @brief Reset the state machine
     */
    void reset() noexcept;

    /**
     * @brief Gets the current tactic name
     * @return Name of the tactic
     */
    virtual const char *getName() = 0;

    /**
     * @brief Gets the skill whose turn it is
     * @return The current skill
     */
    Skill *getCurrentSkill();
};
}  // namespace rtt::ai::stp

#endif  // RTT_TACTIC_H

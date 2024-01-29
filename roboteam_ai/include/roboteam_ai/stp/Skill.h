//
// Created by john on 3/2/20.
//

#ifndef RTT_SKILL_H
#define RTT_SKILL_H

#include <roboteam_utils/RobotCommands.hpp>

#include "stp/StpInfo.h"
#include "world/views/RobotView.hpp"

namespace rtt::ai::stp {

/**
 * @brief Base skill class, inherit from it for making your own skill
 */
class Skill {
   protected:
    Status currentStatus; /**< Status of the last time update() was called */

    rtt::RobotCommand command; /**< Robot command that will eventually be sent to the robot */

    std::optional<world::view::RobotView> robot; /**< Robot this skill controls */

    /**
     * @brief Forwards the current robot command to the ControlModule and refreshes it
     */
    virtual void forwardRobotCommand() noexcept;

    /**
     * @brief Resets the internal robot command
     */
    virtual void refreshRobotCommand() noexcept;

    /**
     * @brief Function that's called when the skill gets updated (every tick)
     * @param info StpInfo structure that provides data to the skill
     * @return Status according to its current execution
     */
    virtual Status onUpdate(StpInfo const& info) noexcept = 0;

   public:
    /**
     * @brief Gets the status from the last time update() was called
     * @return this->currentStatus
     */
    [[nodiscard]] Status getStatus() const;

    /**
     * @brief Calls onInitialize
     */
    virtual void initialize() noexcept;

    /**
     * @brief Function that's called when the skill gets updated (every tick)
     * @param info StpInfo structure that provides data to the skill
     * @return Status according to its current execution
     */
    virtual Status update(StpInfo const& info) noexcept;

    /**
     * @brief Calls onTerminate
     * @return Status of termination
     */
    virtual void terminate() noexcept;

    /**
     * @brief Virtual destructor that ensures proper destruction
     */
    virtual ~Skill() = default;

    /**
     * @brief Gets the current skill name
     * @return Name of the skill
     */
    virtual const char* getName() = 0;
};
}  // namespace rtt::ai::stp

#endif  // RTT_SKILL_H

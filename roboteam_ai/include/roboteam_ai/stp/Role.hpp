//
// Created by john on 3/9/20.
//

#ifndef RTT_ROLE_HPP
#define RTT_ROLE_HPP

#include <vector>

#include "Tactic.h"

namespace rtt::ai::stp {
/**
 * @brief Role class used in STP
 * Every robot needs to have a role
 * A role is essentially a state machine of Tactics
 */
class Role {
   public:
    /**
     * @brief Constructor that has a name as its parameter
     * @param name the name of the role
     */
    explicit Role(std::string name) : roleName{std::move(name)} {}

    /**
     * @brief Function that's called every tick, default implementation is robotTactics.update();
     * @param info TacticInfo to be passed to update()
     * @return The status that the current tactic returns
     */
    [[nodiscard]] virtual Status update(StpInfo const& info) noexcept;

    /**
     * @brief Checks whether the role is finished with all tactics
     * @return True if all tactics returned Status::finish
     */
    [[nodiscard]] bool finished() const noexcept;

    /**
     * @brief Gets the name
     * @return name of the role
     */
    std::string getName() { return roleName; }

    /**
     * @brief Gets the current robot
     * @return view to the robot this role belongs to, optional.
     */
    [[nodiscard]] std::optional<world::view::RobotView> const& getCurrentRobot() const;

    /**
     * @brief Gets the tactic whose turn it is
     * @return Tactic*
     */
    [[nodiscard]] Tactic* getCurrentTactic();

    /**
     * @brief Forces the Role to skip to the next tactic in the state machine
     */
    void forceNextTactic() noexcept;

    /**
     * @brief Forces the Role to skip to the last tactic in the state machine
     */
    void forceLastTactic() noexcept;

    /**
     * @brief Resets the tactics, skills and robot of this role so re-dealing of robots works as expected
     */
    void reset() noexcept;

    /**
     * @brief Virtual default destructor, ensures proper destruction of Role
     */
    virtual ~Role() = default;

    /**
     * @brief Default copy constructor, ensures proper copy construction of Role
     */
    Role(Role& other) = default;

    /**
     * @brief Default mv constructor, ensures proper move construction of Role
     */
    Role(Role&& other) = default;

   protected:
    std::optional<world::view::RobotView> currentRobot; /**< Robot to which this role is currently assigned */

    std::string roleName{}; /**< Name of the role */

    collections::state_machine<Tactic, Status, StpInfo> robotTactics; /**< State machine that keeps track of tactic states */
};
}  // namespace rtt::ai::stp

#endif  // RTT_ROLE_HPP

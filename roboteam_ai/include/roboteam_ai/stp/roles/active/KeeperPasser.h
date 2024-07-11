#ifndef RTT_KEEPERPASSER_H
#define RTT_KEEPERPASSER_H

#include "stp/Role.hpp"

namespace rtt::ai::stp::role {
/**
 * @brief Class that defines the KeeperPasser role. The keeper will try to pass the ball to another robot
 */
class KeeperPasser : public Role {
   public:
    /**
     * @brief Constructor that sets the name of the role and creates a state machine of tactics
     * @param name The name of the role
     */
    KeeperPasser(std::string name);

    /**
     * @brief Besides the default update from base class Role, it also switches between tactics depending on the ball position
     * @param info TacticInfo to be passed to update()
     * @return The status that the current tactic returns
     */
    [[nodiscard]] Status update(StpInfo const& info) noexcept override;
};
}  // namespace rtt::ai::stp::role

#endif  // RTT_KEEPERPASSER_H

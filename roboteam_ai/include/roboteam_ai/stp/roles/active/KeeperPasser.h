//
// Created by tijmen on 14-07-22.
//

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
};
}  // namespace rtt::ai::stp::role

#endif  // RTT_KEEPERPASSER_H

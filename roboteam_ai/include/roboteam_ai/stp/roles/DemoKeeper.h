//
// Created by Luuk and Jorn on 14/09/2023
//

#ifndef RTT_DEMOKEEPER_H
#define RTT_DEMOKEEPER_H

#include <roboteam_utils/Field.hpp>

#include "stp/Role.hpp"

namespace rtt::ai::stp::role {
/**
 * @brief Class that defines the keeper role. The keeper will block the ball from going into the goal
 */
class DemoKeeper : public Role {
   public:
    /**
     * @brief Constructor that sets the name of the role and creates a state machine of tactics
     * @param name name of the role
     */
    explicit DemoKeeper(std::string name);
};
}  // namespace rtt::ai::stp::role

#endif  // RTT_DEMOKEEPER_H

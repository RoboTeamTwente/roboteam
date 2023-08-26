//
// Created by jordi on 08-04-20.
//

#ifndef RTT_KEEPER_H
#define RTT_KEEPER_H

#include <roboteam_utils/Field.hpp>

#include "stp/Role.hpp"

namespace rtt::ai::stp::role {
/**
 * @brief Class that defines the keeper role. The keeper will block the ball from going into the goal
 */
class Keeper : public Role {
   public:
    /**
     * @brief Constructor that sets the name of the role and creates a state machine of tactics
     * @param name name of the role
     */
    explicit Keeper(std::string name);
};
}  // namespace rtt::ai::stp::role

#endif  // RTT_KEEPER_H

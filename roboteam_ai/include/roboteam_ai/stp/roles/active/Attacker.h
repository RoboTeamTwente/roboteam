//
// Created by jordi on 17-03-20.
//

#ifndef RTT_ATTACKER_H
#define RTT_ATTACKER_H

#include "stp/Role.hpp"

namespace rtt::ai::stp::role {
/**
 * @brief Class that defines the attacker role. The attacker will try to shoot at the enemy goal.
 */
class Attacker : public Role {
   public:
    /**
     * @brief Constructor that sets the name of the role and creates a state machine of tactics
     * @param name The name of the role
     */
    Attacker(std::string name);
};
}  // namespace rtt::ai::stp::role

#endif  // RTT_ATTACKER_H

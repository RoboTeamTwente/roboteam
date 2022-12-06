//
// Created by doormat on 22-11-22.
//

#ifndef RTT_CHIPPER_H
#define RTT_CHIPPER_H

#include "stp/Role.hpp"

namespace rtt::ai::stp::role {

class Chipper : public Role {
   public:
    /**
     * Ctar that sets the name of the role and creates a state machine of tactics
     * @param name name of the role
     */
    Chipper(std::string name);
};
} // namespace rtt::ai::stp::role


#endif  // RTT_CHIPPER_H
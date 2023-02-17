//
// Created by jessevw on 17.03.20.
//

#ifndef RTT_PASSRECEIVER_H
#define RTT_PASSRECEIVER_H

#include "stp/Role.hpp"

namespace rtt::ai::stp::role {
/**
 * @brief Class that defines the pass receiver role. The pass receiver will try to receive the ball from the passer
 */
class PassReceiver : public Role {
   public:
    /**
     * @brief Constructor that sets the name of the role and creates a state machine of tactics
     * @param name The name of the role
     */
    PassReceiver(std::string name);
};
}  // namespace rtt::ai::stp::role

#endif  // RTT_PASSRECEIVER_H

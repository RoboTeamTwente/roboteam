//
// Created by jibbe on 6-6-23.
//

#ifndef RTT_ASSISTANT_HPP
#define RTT_ASSISTANT_HPP

#include "stp/Role.hpp"

namespace rtt::ai::stp::role {
/**
 * @brief Class that defines the pass receiver role. The pass receiver will try to receive the ball from the passer
 */
class Assistant : public Role {
   public:
    /**
     * @brief Constructor that sets the name of the role and creates a state machine of tactics
     * @param name The name of the role
     */
    Assistant(std::string name);
};
}  // namespace rtt::ai::stp::role

#endif  // RTT_ASSISTANT_HPP

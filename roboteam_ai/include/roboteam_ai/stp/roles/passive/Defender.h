//
// Created by jordi on 26-03-20.
//

#ifndef RTT_DEFENDER_H
#define RTT_DEFENDER_H

#include "stp/Role.hpp"

namespace rtt::ai::stp::role {
/**
 * @brief Class that defines the ball defender role. The ball defender will try to block the ball from begin shot to a given position.
 */
class Defender : public Role {
   public:
    /**
     * @brief Constructor that sets the name of the role and creates a state machine of tactics
     * @param name The name of the role
     */
    Defender(std::string name);
};
}  // namespace rtt::ai::stp::role

#endif  // RTT_DEFENDER_H

//
// Created by jesse on 30-04-20.
//

#ifndef RTT_BALLAVOIDER_H
#define RTT_BALLAVOIDER_H

#include "stp/Role.hpp"

namespace rtt::ai::stp::role {
/**
 * Class that defines the ball avoider role. The ball avoider keep at least the given distance to the ball
 */
class BallAvoider : public Role {
   public:
    /**
     * @brief Constructor that sets the name of the role and creates a state machine of tactics
     * @param name The name of the role
     */
    BallAvoider(std::string name);
};
}  // namespace rtt::ai::stp::role

#endif  // RTT_BALLAVOIDER_H

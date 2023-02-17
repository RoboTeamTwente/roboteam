//
// Created by tijmen on 24-04-22.
//

#ifndef RTT_HARASSER_H
#define RTT_HARASSER_H

#include "stp/Role.hpp"

namespace rtt::ai::stp::role {
/**
 * @brief Class that defines the harasser roll. The harasser will try to steal the ball from the opponent.
 */
class Harasser : public Role {
   public:
    /**
     * @brief Constructor that sets the name of the role and creates a state machine of tactics
     * @param name The name of the role
     */
    Harasser(std::string name);
};
}  // namespace rtt::ai::stp::role

#endif  // RTT_HARASSER_H

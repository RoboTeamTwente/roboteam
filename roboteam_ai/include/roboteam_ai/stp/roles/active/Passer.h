#ifndef RTT_PASSER_H
#define RTT_PASSER_H

#include "stp/Role.hpp"

namespace rtt::ai::stp::role {
/**
 * @brief Class that defines the passer role. The passer will try to pass the ball to another robot.
 */
class Passer : public Role {
   public:
    /**
     * @brief Constructor that sets the name of the role and creates a state machine of tactics
     * @param name The name of the role
     */
    Passer(std::string name);
};
}  // namespace rtt::ai::stp::role

#endif  // RTT_PASSER_H

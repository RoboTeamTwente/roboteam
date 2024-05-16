#ifndef RTT_CHIPPER_H
#define RTT_CHIPPER_H

#include "stp/Role.hpp"

namespace rtt::ai::stp::role {
/**
 * @brief Class that defines the Chipper role. The Chipper will chip the ball to another robot.
 */
class Chipper : public Role {
   public:
    /**
     * @brief Constructor that sets the name of the role and creates a state machine of tactics
     * @param name name of the role
     */
    Chipper(std::string name);
};
}  // namespace rtt::ai::stp::role

#endif  // RTT_CHIPPER_H
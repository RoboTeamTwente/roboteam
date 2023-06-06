//
// Created by timovdk on 3/27/20.
//

#ifndef RTT_ROBOTDEFENDER_H
#define RTT_ROBOTDEFENDER_H

#include "stp/Role.hpp"

namespace rtt::ai::stp::role {
/**
 * @brief Class that defines the robot defender role. The robot defender will try to block an enemy robot from moving to its desired position
 */
class RobotDefender : public Role {
   public:
    /**
     * @brief Constructor that sets the name of the role and creates a state machine of tactics
     * @param name name of the role
     */
    RobotDefender(std::string name);
};
}  // namespace rtt::ai::stp::role

#endif  // RTT_ROBOTDEFENDER_H

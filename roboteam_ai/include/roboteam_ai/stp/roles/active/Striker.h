#ifndef RTT_STRIKER_H
#define RTT_STRIKER_H

#include "stp/Role.hpp"

namespace rtt::ai::stp::role {
/**
 * @brief Class that defines the striker role. The striker will try to shoot at the enemy goal.
 */
class Striker : public Role {
   public:
    /**
     * @brief Constructor that sets the name of the role and creates a state machine of tactics
     * @param name The name of the role
     */
    Striker(std::string name);
};
}  // namespace rtt::ai::stp::role

#endif  // RTT_STRIKER_H

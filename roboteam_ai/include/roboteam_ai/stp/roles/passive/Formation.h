#ifndef RTT_FORMATION_H
#define RTT_FORMATION_H

#include "stp/Role.hpp"

namespace rtt::ai::stp::role {
/**
 * @brief Class that defines the formation role. the formation will keep a given position until it is given a different role
 */
class Formation : public Role {
   public:
    /**
     * @brief Constructor that sets the name of the role and creates a state machine of tactics
     * @param name The name of the role
     */
    Formation(std::string name);
};
}  // namespace rtt::ai::stp::role

#endif  // RTT_FORMATION_H

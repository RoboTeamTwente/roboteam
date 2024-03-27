#ifndef RTT_TESTROLE_H
#define RTT_TESTROLE_H

#include "stp/Role.hpp"

namespace rtt::ai::stp {

class TestRole : public Role {
    /**
     * @brief Class that defines the test role. The test role is used for testing the code
     */
   public:
    /**
     * @brief Constructor that sets the name of the role and creates a statemachine of tactics
     * @param name name of the role
     */
    TestRole(std::string name);
};
}  // namespace rtt::ai::stp

#endif  // RTT_TESTROLE_H

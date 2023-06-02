#ifndef RTT_PENALTYTAKER_H
#define RTT_PENALTYTAKER_H

#include "stp/Role.hpp"

namespace rtt::ai::stp::role {
/**
 * @brief Class that defines the penalty taker role. The penalty taker will try to shoot at the enemy goal during a penalty.
 */
class PenaltyTaker : public Role {
   public:
    /**
     * @brief Constructor that sets the name of the role and creates a state machine of tactics
     * @param name The name of the role
     */
    PenaltyTaker(std::string name);
};
}  // namespace rtt::ai::stp::role

#endif  // RTT_PENALTYTAKER_H

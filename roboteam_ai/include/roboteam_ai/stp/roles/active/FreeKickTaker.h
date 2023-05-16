//
// Created Alexander on 22-04-2022
//

#ifndef RTT_FREEKICKTAKER_H
#define RTT_FREEKICKTAKER_H

#include "stp/Role.hpp"

namespace rtt::ai::stp::role {
/**
 * @brief Class that defines the free kick taker role. The free kick taker will take the free kick and either pass to another robot or shoot at the goal.
 */
class FreeKickTaker : public Role {
   public:
    /**
     * @brief Constructor that sets the name of the role and creates a state machine of tactics
     * @param name The name of the role
     */
    FreeKickTaker(std::string name);
};
}  // namespace rtt::ai::stp::role

#endif  // RTT_FREEKICKTAKER_H

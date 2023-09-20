//
// Created by Luuk and Jorn on 12/09/2023.
//

#ifndef RTT_KEEPERDEMO_H
#define RTT_KEEPERDEMO_H

#include "stp/Play.hpp"

namespace rtt::ai::stp::play {

/**
 * @brief KeeperDemo Play is executed only during the demo. It is used to show the keeper's capabilities
 */

class KeeperDemo : public Play {
   public:
    /**
     * @brief Constructor that initializes roles with roles that are necessary for this play
     */
    KeeperDemo();

    /**
     * @brief Calculates the score of this play to determine which play is best in this situation
     * @param field The current Field
     * @return The score of this play (0-255)
     */
    uint8_t score(const rtt::Field& field) noexcept override;

    /**
     * @brief Assigns robots to roles of this play
     * @return Map with assigned roles
     */
    Dealer::FlagMap decideRoleFlags() const noexcept override;

    /**
     * @brief Calculates info for the roles
     */
    void calculateInfoForRoles() noexcept override;

    /**
     * @brief Retrieve name of the play
     * @return The name of the play as a string
     */
    const char* getName() const override;

};
}  // namespace rtt::ai::stp::play

#endif  // RTT_KEEPERDEMO_H

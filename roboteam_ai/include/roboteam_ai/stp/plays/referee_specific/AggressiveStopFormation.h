//
// Created by timo on 3/30/20.
//

#ifndef RTT_AGGRESSIVESTOPFORMATION_H
#define RTT_AGGRESSIVESTOPFORMATION_H

#include "stp/Play.hpp"

namespace rtt::ai::stp::play {

/**
 * @brief The aggressive stop play is executed when we want to attack after the stop game state has ended.
 */
class AggressiveStopFormation : public Play {
   public:
    /**
     * @brief Constructor that initializes roles with roles that are necessary for this play
     */
    AggressiveStopFormation();

    /**
     * @brief Calculates the score of this play to determine which play is best in this situation
     * @param field The current Field
     * @return The score of this play (0-255)
     */
    uint8_t score(const rtt::Field& field) noexcept override;

    /**
     * @brief Assigns robots to roles of this play
     * @return A map with assigned roles
     */
    Dealer::FlagMap decideRoleFlags() const noexcept override;

    /**
     * Calculates info for the roles
     */
    void calculateInfoForRoles() noexcept override;

    /**
     * @brief Retrieves the name of the play
     * @return The name of the play as string
     */
    const char* getName() override;
};
}  // namespace rtt::ai::stp::play

#endif  // RTT_AGGRESSIVESTOPFORMATION_H

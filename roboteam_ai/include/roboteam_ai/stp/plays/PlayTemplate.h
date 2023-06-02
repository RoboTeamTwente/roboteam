//
// Created by maxl on 29-03-21.
/// THIS IS MEANT AS A TEMPLATE FOR MAKING PLAYS. DO NOT ADD THIS FILE TO CMAKE.
//

#ifndef RTT_PLAYTEMPLATE_H
#define RTT_PLAYTEMPLATE_H

#include "stp/Play.hpp"

namespace rtt::ai::stp::play {
/**
 * @brief Template class for plays.
 * Add specific information about the play here
 */
class PlayTemplate : public Play {
   public:
    /**
     * @brief Constructor that initializes roles with roles that are necessary for this play
     */
    PlayTemplate();

    /**
     * @brief Calculates the score of this play to determine which play is best in this situation
     * @param field The current Field
     * @return The score of this play (0-255)
     */
    uint8_t score(const rtt::Field &field) noexcept override;

    /**
     * @brief Assigns robots to roles of this play
     * @return A map with assigned roles
     */
    Dealer::FlagMap decideRoleFlags() const noexcept override;

    /**
     * @brief Calculates info for the roles
     */
    void calculateInfoForRoles() noexcept override;

    /**
     * @brief Calculate info for the roles that need to be calculated for scoring
     */
    void calculateInfoForScoredRoles(world::World *) noexcept override{};

    /**
     * @brief Retrieves the name of the play
     * @return The name of the play as a string
     */
    const char *getName() const override;

    /**
     * @brief Optional function to force end a play
     * @return Boolean that tells whether the play should end
     */
    bool shouldEndPlay() noexcept override;
};
}  // namespace rtt::ai::stp::play

#endif  // RTT_PLAYTEMPLATE_H

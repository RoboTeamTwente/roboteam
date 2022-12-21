//
// Created by timovdk on 5/1/20.
//

#ifndef RTT_KICKOFFTHEM_H
#define RTT_KICKOFFTHEM_H

#include "stp/Play.hpp"

namespace rtt::ai::stp::play {
/**
 * @brief The kick off them play is executed when the kick off them game state is selected
 */
class KickOffThem : public Play {
   public:
    /**
     * @brief Constructor that initializes roles with roles that are necessary for this play
     */
    KickOffThem();

    /**
     * @brief Calculates the score of this play to determine which play is best in this situation
     * @param field The current Field
     * @return The score of this play (0-255)
     */
    uint8_t score(const rtt::world::Field& field) noexcept override;

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
    void calculateInfoForScoredRoles(world::World*) noexcept override{};

    /**
     * @brief Check if the play should end. True after kickoff
     * @return Boolean that tells whether the play should end
     */
    bool shouldEndPlay() noexcept override;

    /**
     * @brief Retrieves the name of the play
     * @return The name of the play as string
     */
    const char* getName() override;
};
}  // namespace rtt::ai::stp::play

#endif  // RTT_KICKOFFTHEM_H

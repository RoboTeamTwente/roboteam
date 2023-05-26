//
// Created by timovdk on 5/1/20.
//

#ifndef RTT_KICKOFFUS_H
#define RTT_KICKOFFUS_H

#include "stp/Play.hpp"

namespace rtt::ai::stp::play {
/**
 * @brief The kick off us play is executed when the kick off us game state is selected
 */
class KickOffUs : public Play {
   public:
    /**
     * @brief Constructor that initializes roles with roles that are necessary for this play
     */
    KickOffUs();

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
    const char* getName() const override;

    /**
     * @brief Checks if the passer has finished kickAtPos
     * @return Boolean that tells whether the ball has been kicked
     */
    bool ballKicked();
};
}  // namespace rtt::ai::stp::play

#endif  // RTT_KICKOFFUS_H

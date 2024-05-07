#ifndef RTT_FREE_KICK_US_PASS_PLAY_H
#define RTT_FREE_KICK_US_PASS_PLAY_H

#include <roboteam_utils/Grid.h>

#include "stp/Play.hpp"
#include "stp/computations/PassComputations.h"

namespace rtt::ai::stp::play {
/**
 * @brief The free kick us pass play is executed when we want to pass and the free kick us game state is selected
 */
class FreeKickUsPass : public Play {
   public:
    /**
     * @brief Constructor that initializes roles with roles that are necessary for this play
     */
    FreeKickUsPass();

    /**
     * @brief Calculate how beneficial we expect this play to be
     * @param field The current field
     * @return score of the play (0-255)
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
     * @brief Retrieves the name of the play
     * @return The name of the play as a string
     */
    const char* getName() const override;

    /**
     * @brief Check if play should end. True if pass arrived, if the ball is not moving anymore after pass, or if there is a better pass available
     * @return Boolean that tells whether the play should end
     */
    bool shouldEndPlay() noexcept override;

   private:
    /**
     * @brief Return true if passer is done with kicking
     * @return Boolean that tells whether the ball has been kicked
     */
    bool ballKicked();

    PassInfo passInfo; /**< Struct containing info about the pass. Calculated once for each time this play is run */
};
}  // namespace rtt::ai::stp::play

#endif  // RTT_FREE_KICK_US_PASS_PLAY_H

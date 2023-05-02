//
// Created by jessevw on 17.03.20.
//

#ifndef RTT_ATTACKING_PASS_PLAY_H
#define RTT_ATTACKING_PASS_PLAY_H

#include <roboteam_utils/Grid.h>

#include "stp/Play.hpp"
#include "stp/computations/PassComputations.h"

namespace rtt::ai::stp::play {

/**
 * @brief The attacking pass play is executed when we want to pass the ball to a robot that can shoot at the enemy goal
 */
class AttackingPass : public Play {
   public:
    /**
     * @brief Constructor that initializes roles with roles that are necessary for this play
     */
    AttackingPass();

    /**
     *  @brief Calculate how beneficial we expect this play to be
     *  @param field The current field
     *  @return The score of the play
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
     * @brief Calculates info for the defenders
     */
    void calculateInfoForDefenders() noexcept;

    /**
     * @brief Calculates info for the attackers
     */
    void calculateInfoForAttackers() noexcept;

    /**
     * @brief Calculate info for the roles that need to be calculated for scoring
     * @param world The current world
     */
    void calculateInfoForScoredRoles(world::World*) noexcept override{};

    /**
     * @brief Retrieves the name of the play
     * @return The name of the play as string
     */
    const char* getName() override;

    /**
     * @brief Check if play should end. True if pass arrived, if the ball is not moving anymore after pass, or if there is a better pass available
     * @return Boolean that tells whether the play should end
     */
    bool shouldEndPlay() noexcept override;

   private:
    /**
     * @brief Check if the ball has been kicked. True if passer is done with KickAtPos tactic, False if it is still ongoing
     * @return Boolean that tells whether the ball has been kicked
     */
    bool ballKicked();

    PassInfo passInfo; /**< Struct containing info about the pass. Calculated once for each time this play is run */

    /**
     * @brief Calculate info for the blocker roles
     */
    void calculateInfoForBlocker () noexcept;
};
}  // namespace rtt::ai::stp::play

#endif  // RTT_ATTACKING_PASS_PLAY_H

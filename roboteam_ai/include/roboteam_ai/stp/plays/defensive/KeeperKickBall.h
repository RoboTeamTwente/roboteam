//
// Created by Alexander on 29/01/2022.
//

#ifndef RTT_KEEPERKICKBALL_H
#define RTT_KEEPERKICKBALL_H

#include "stp/Play.hpp"
#include "stp/computations/PassComputations.h"

namespace rtt::ai::stp::play {

/**
 * @brief KeeperKickBall Play is executed when the ball is in our defense area and should be kicked out by our keeper
 */

class KeeperKickBall : public Play {
   public:
    /**
     * @brief Constructor that initializes roles with roles that are necessary for this play
     */
    KeeperKickBall();

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
     * @brief Calculates info for the defenders
     */
    void calculateInfoForDefenders() noexcept;

    /**
     * @brief Calculates info for the midfielders
     */
    void calculateInfoForMidfielders() noexcept;

    /**
     * @brief Calculates info for the attackers
     */
    void calculateInfoForAttackers() noexcept;

    /**
     * @brief Calculate info for the roles that need to be calculated for scoring
     */
    void calculateInfoForScoredRoles(world::World* world) noexcept override{};

    /**
     * @brief Retrieve name of the play
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
     * @brief Check if the ball has been kicked. True if the kick tactic has ended, false if it is still going.
     * @return Boolean that tells whether the ball has been kicked
     */
    bool ballKicked();

    PassInfo passInfo; /**< Struct containing info about the pass. Calculated once for each time this play is run */
};
}  // namespace rtt::ai::stp::play

#endif  // RTT_KEEPERKICKBALL_H

//
// Created by doormat on 22-11-22.
//

#ifndef RTT_CHIPPINGPASS_H
#define RTT_CHIPPINGPASS_H

#include <roboteam_utils/Grid.h>

#include "stp/Play.hpp"
#include "stp/computations/PassComputations.h"

namespace rtt::ai::stp::play {
/**
 * @brief The chipping pass play is executed when we want to pass the ball to a robot that can not be passed to directly
 */
class ChippingPass : public Play {
   public:
    /**
     * @brief Constructor that initializes roles with roles that are necessary for this play
     */
    ChippingPass();

    /**
     * @brief Calculate how beneficial we expect this play to be
     * @param field The current field
     * @return A score of this play
     */
    uint8_t score(const rtt::Field& field) noexcept override;

    /**
     * @brief Assigns robots to roles of this play
     * @return A mapping of dealer flags per role
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
     * @brief Gets the name of the play
     * @return The name of the play
     */
    const char* getName() const override;

    /**
     * @brief Check if play should end. True if pass arrived, if the ball is not moving anymore after pass, or if there is a better pass available
     * @return Boolean that indicates whether the play should end
     */
    bool shouldEndPlay() noexcept override;

   private:
    /**
     * @brief Return true if passer is done with ChipAtPos tactic
     * @return Boolean that indicates whether the passer is done with the ChipAtPos tactic
     */
    bool ballKicked();

    PassInfo passInfo; /**< Struct containing info about the pass. Calculated once for each time this play is run */

    /**
     * @brief Calculate info for the blocker roles
     */
    void calculateInfoForBlocker() noexcept;
};
}  // namespace rtt::ai::stp::play

#endif  // RTT_CHIPPINGPASS_H

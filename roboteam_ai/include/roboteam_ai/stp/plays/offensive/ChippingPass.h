//
// Created by doormat on 22-11-22.
//

#ifndef RTT_CHIPPINGPASS_H
#define RTT_CHIPPINGPASS_H

#include <roboteam_utils/Grid.h>

#include "stp/Play.hpp"
#include "stp/computations/PassComputations.h"

namespace rtt::ai::stp::play {

class ChippingPass : public Play {
   public:
    /**
     * Constructor that initializes roles with roles that are necessary for this play
     */
    ChippingPass();

    /**
     *  Calculate how beneficial we expect this play to be
     */
    uint8_t score(const rtt::Field& field) noexcept override;

    /**
     * Assigns robots to roles of this play
     */
    Dealer::FlagMap decideRoleFlags() const noexcept override;

    /**
     * Calculates info for the roles
     */
    void calculateInfoForRoles() noexcept override;

    /**
     * Calculates info for the defenders
     */
    void calculateInfoForDefenders() noexcept;

    /**
     * Calculates info for the midfielders
     */
    void calculateInfoForMidfielders() noexcept;

    /**
     * Calculates info for the attackers
     */
    void calculateInfoForAttackers() noexcept;

    /**
     * Gets the play name
     */
    const char* getName() override;

    /**
     * Check if play should end. True if pass arrived, if the ball is not moving anymore after pass, or if there is a better pass available
     */
    bool shouldEndPlay() noexcept override;

   private:
    /**
     * Return true if passer is done with ChipAtPos tactic
     */
    bool ballKicked();

    /**
     * Struct containing info about the pass. Calculated once for each time this play is run
     */
    PassInfo passInfo;

    void calculateInfoForBlocker () noexcept;
};
}  // namespace rtt::ai::stp::play

#endif  // RTT_CHIPPINGPASS_H

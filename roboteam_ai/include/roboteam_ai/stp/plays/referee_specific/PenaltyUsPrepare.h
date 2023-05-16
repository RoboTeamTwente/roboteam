//
// Created by timovdk on 5/1/20.
//

#ifndef RTT_PENALTYUSPREPARE_H
#define RTT_PENALTYUSPREPARE_H

#include "stp/Play.hpp"

namespace rtt::ai::stp::play {
/**
 * @brief The penalty us prepare play is executed when the penalty us prepare game state is selected
 */
class PenaltyUsPrepare : public Play {
   public:
    /**
     * @brief Constructor that initializes roles with roles that are necessary for this play
     */
    PenaltyUsPrepare();

    /**
     * @brief Calculates the score of this play to determine which play is best in this situation
     * @param field The current Field
     * @return The score of this play (0-255)
     */
    uint8_t score(const rtt::Field& field) noexcept override;

    /**
     * @brief Assigns robots to roles of this play
     * @return A map with assigne roles
     */
    Dealer::FlagMap decideRoleFlags() const noexcept override;

    /**
     * @brief Calculates info for the roles
     */
    void calculateInfoForRoles() noexcept override;

    /**
     * @brief Info that should be calculated for scoring of the play
     */
    void calculateInfoForScoredRoles(world::World*) noexcept override{};

    /**
     * @brief Retrieves the name of the play
     * @return The name of the play as a string
     */
    const char* getName() override;
};
}  // namespace rtt::ai::stp::play

#endif  // RTT_PENALTYUSPREPARE_H

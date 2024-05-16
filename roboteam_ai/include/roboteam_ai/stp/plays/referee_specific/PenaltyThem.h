#ifndef RTT_PENALTYTHEM_H
#define RTT_PENALTYTHEM_H

#include "stp/Play.hpp"

namespace rtt::ai::stp::play {
/**
 * @brief The penalty them play is executed when the penalty them game state is selected
 */
class PenaltyThem : public Play {
   public:
    /**
     * @brief Constructor that initializes roles with test roles
     */
    PenaltyThem();

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
     * @brief Retrieves the name of the play
     * @return The name of the play as a string
     */
    const char* getName() const override;
};
}  // namespace rtt::ai::stp::play

#endif  // RTT_PENALTYTHEM_H

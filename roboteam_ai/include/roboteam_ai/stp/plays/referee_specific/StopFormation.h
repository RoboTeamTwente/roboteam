#ifndef RTT_STOPFORMATION_H
#define RTT_STOPFORMATION_H

#include "stp/Play.hpp"

namespace rtt::ai::stp::play {
/**
 * @brief The stop formation is executed when a stop state is called
 */
class StopFormation : public Play {
   public:
    /**
     * @brief Constructor that initializes roles with roles that are necessary for this play
     */
    StopFormation();

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

    InterceptionInfo harasserInfo;
};
}  // namespace rtt::ai::stp::play

#endif  // RTT_STOPFORMATION_H

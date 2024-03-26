#ifndef RTT_DEFENDPASS_H
#define RTT_DEFENDPASS_H

#include "stp/Play.hpp"

namespace rtt::ai::stp::play {

/**
 * @brief DefendPass Play is executed when the opponent has or is close to the ball but not necessarily on our side of the field.
 * In this case the opponent most likely will pass to another robot. Our robots will namely block off robots that can
 * be passed to.
 */
class DefendPass : public Play {
   public:
    /**
     * @brief Constructor that initializes roles with roles that are necessary for this play
     */
    DefendPass();

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
     * @brief Retrieves the name of the play
     * @return The name of the play as string
     */
    const char* getName() const override;

    InterceptInfo harasserInfo;
};
}  // namespace rtt::ai::stp::play

#endif  // RTT_DEFENDPASS_H

#ifndef RTT_KICKOFFUSPREPAREGAMESTATEEVALUATION_H
#define RTT_KICKOFFUSPREPAREGAMESTATEEVALUATION_H

#include "stp/evaluations/BaseEvaluation.h"

namespace rtt::ai::stp::evaluation {
/**
 * @brief Class that evaluates the kick of us prepare game state
 */
class KickOffUsPrepareGameStateEvaluation : public BaseEvaluation {
   public:
    /**
     * @brief Calculates the score for the kick of us prepare game state
     * @param world The current world
     * @param field The current field
     * @return The score of the kick of us prepare game state
     */
    [[nodiscard]] uint8_t metricCheck(const world::World* world, const Field* field) const noexcept override;

    /**
     * @brief Retrieves the name of the evaluation
     * @return A string containing the name of the evaluation
     */
    const char* getName() override { return "gs::KickOffUsPrepare"; }
};
}  // namespace rtt::ai::stp::evaluation

#endif  // RTT_KICKOFFUSPREPAREGAMESTATEEVALUATION_H

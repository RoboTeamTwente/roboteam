#ifndef RTT_PENALTYUSPREPAREGAMESTATEEVALUATION_H
#define RTT_PENALTYUSPREPAREGAMESTATEEVALUATION_H

#include "stp/evaluations/BaseEvaluation.h"

namespace rtt::ai::stp::evaluation {
/**
 * @brief Class that evaluates the free kick them game state
 */
class PenaltyUsPrepareGameStateEvaluation : public BaseEvaluation {
   public:
    /**
     * @brief Calculates the score for the free kick them game state
     * @param world The current world
     * @param field The current field
     * @return The score of the free kick them game state
     */
    [[nodiscard]] uint8_t metricCheck(const world::World* world, const Field* field) const noexcept override;

    /**
     * @brief Retrieves the name of the evaluation
     * @return A string containing the name of the evaluation
     */
    const char* getName() override { return "gs::PenaltyUsPrepare"; }
};
}  // namespace rtt::ai::stp::evaluation

#endif  // RTT_PENALTYUSPREPAREGAMESTATEEVALUATION_H

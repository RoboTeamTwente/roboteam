#ifndef RTT_PENALTYTHEMGAMESTATEEVALUATION_H
#define RTT_PENALTYTHEMGAMESTATEEVALUATION_H

#include "stp/evaluations/BaseEvaluation.h"

namespace rtt::ai::stp::evaluation {
/**
 * @brief Class that evaluates the penalty them game state
 */
class PenaltyThemGameStateEvaluation : public BaseEvaluation {
   public:
    /**
     * @brief Calculates the score for the penalty them game state
     * @param world The current world
     * @param field The current field
     * @return The score of the penalty them game state
     */
    [[nodiscard]] uint8_t metricCheck(const world::World* world, const Field* field) const noexcept override;

    /**
     * @brief Retrieves the name of the evaluation
     * @return A string containing the name of the evaluation
     */
    const char* getName() override { return "gs::PenaltyThem"; }
};
}  // namespace rtt::ai::stp::evaluation

#endif  // RTT_PENALTYTHEMGAMESTATEEVALUATION_H

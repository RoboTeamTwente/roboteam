#ifndef RTT_HALTGAMESTATEEVALUATION_H
#define RTT_HALTGAMESTATEEVALUATION_H

#include "stp/evaluations/BaseEvaluation.h"

namespace rtt::ai::stp::evaluation {
/**
 * @brief Class that evaluates the halt game state
 */
class HaltGameStateEvaluation : public BaseEvaluation {
   public:
    /**
     * @brief Calculates the score for the halt game state
     * @param world The current world
     * @param field The current field
     * @return The score of the halt game state
     */
    [[nodiscard]] uint8_t metricCheck(const world::World* world, const Field* field) const noexcept override;

    /**
     * @brief Retrieves the name of the evaluation
     * @return A string containing the name of the evaluation
     */
    const char* getName() override { return "gs::Halt"; }
};
}  // namespace rtt::ai::stp::evaluation

#endif  // RTT_HALTGAMESTATEEVALUATION_H

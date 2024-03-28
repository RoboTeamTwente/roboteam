#ifndef RTT_NORMALPLAYGAMESTATEEVALUATION_H
#define RTT_NORMALPLAYGAMESTATEEVALUATION_H

#include "stp/evaluations/BaseEvaluation.h"

namespace rtt::ai::stp::evaluation {
/**
 * @brief Class that evaluates the normal play game state
 */
class NormalPlayGameStateEvaluation : public BaseEvaluation {
   public:
    /**
     * @brief Calculates the score for the normal play game state
     * @param world The current world
     * @param field The current field
     * @return The score of the normal play game state
     */
    [[nodiscard]] uint8_t metricCheck(const world::World* world, const Field* field) const noexcept override;

    /**
     * @brief Retrieves the name of the evaluation
     * @return A string containing the name of the evaluation
     */
    const char* getName() override { return "gs::NormalPlay"; }
};
}  // namespace rtt::ai::stp::evaluation

#endif  // RTT_NORMALPLAYGAMESTATEEVALUATION_H

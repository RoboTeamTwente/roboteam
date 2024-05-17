#ifndef RTT_PREPAREFORCEDSTARTGAMESTATEEVALUATION_H
#define RTT_PREPAREFORCEDSTARTGAMESTATEEVALUATION_H

#include "stp/evaluations/BaseEvaluation.h"

namespace rtt::ai::stp::evaluation {
/**
 * @brief Class that evaluates the prepare forced start game state
 */
class PrepareForcedStartGameStateEvaluation : public BaseEvaluation {
   public:
    /**
     * @brief Calculates the score for the prepare forced start game state
     * @param world The current world
     * @param field The current field
     * @return The score of the prepare forced start game state
     */
    [[nodiscard]] uint8_t metricCheck(const world::World* world, const Field* field) const noexcept override;

    /**
     * @brief Retrieves the name of the evaluation
     * @return A string containing the name of the evaluation
     */
    const char* getName() override { return "gs::PrepareForcedStart"; }
};
}  // namespace rtt::ai::stp::evaluation

#endif  // RTT_PREPAREFORCEDSTARTGAMESTATEEVALUATION_H

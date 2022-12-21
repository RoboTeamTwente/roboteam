//
// Created by Alexander on 28-01-2022
//

#ifndef RTT_KICKOFFUSORNORMALGAMESTATEEVALUATION_H
#define RTT_KICKOFFUSORNORMALGAMESTATEEVALUATION_H

#include "stp/evaluations/BaseEvaluation.h"

namespace rtt::ai::stp::evaluation {
/**
 * @brief Class that evaluates the kick off us or normal game state
 */
class KickOffUsOrNormalGameStateEvaluation : public BaseEvaluation {
   public:
    /**
     * @brief Calculates the score for the kick off us or normal game state
     * @param world The current world
     * @param field The current field
     * @return The score of the kick off us or normal game state
     */
    [[nodiscard]] uint8_t metricCheck(const world::World* world, const world::Field* field) const noexcept override;
    
    /**
     * @brief Retrieves the name of the evaluation
     * @return A string containing the name of the evaluation
     */
    const char* getName() override { return "gs::KickOffUsOrNormal"; }
};
}  // namespace rtt::ai::stp::evaluation

#endif  // RTT_KICKOFFUSORNORMALGAMESTATEEVALUATION_H

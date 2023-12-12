//
// Created by Luuk and Jorn on 12-12-2023
//

#ifndef RTT_WEWANTTODEFENDGLOBALEVALUATION_H
#define RTT_WEWANTTODEFENDGLOBALEVALUATION_H

#include "stp/evaluations/BaseEvaluation.h"

namespace rtt::ai::stp::evaluation {
/**
 * @brief Class that evaluates if we want to defend, based on the position of the ball with regards to our goal
 */
class WeWantToDefendGlobalEvaluation : public BaseEvaluation {
   public:
    /**
     * @brief Calculates the score for whether we want to defend
     * @param world The current world
     * @param field The current field
     * @return The score of we want to defend
     */
    [[nodiscard]] uint8_t metricCheck(const world::World* world, const Field* field) const noexcept override;

    /**
     * @brief Retrieves the name of the evaluation
     * @return A string containing the name of the evaluation
     */
    const char* getName() override { return "WeWantToDefend"; }
};
}  // namespace rtt::ai::stp::evaluation

#endif  // RTT_WEWANTTODEFENDGLOBALEVALUATION_H

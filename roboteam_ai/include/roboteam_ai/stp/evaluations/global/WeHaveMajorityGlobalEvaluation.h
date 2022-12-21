//
// Created by luukkn on 21-04-20.
//

#ifndef RTT_WEHAVEMAJORITYGLOBALEVALUATION_H
#define RTT_WEHAVEMAJORITYGLOBALEVALUATION_H

#include "stp/evaluations/BaseEvaluation.h"

namespace rtt::ai::stp::evaluation {
/**
 * @brief Class that evaluates we have majority
 */
class WeHaveMajorityGlobalEvaluation : public BaseEvaluation {
   public:
    /**
     * @brief Calculates the score for we have majority
     * @param world The current world
     * @param field The current field
     * @return The score of we have majority
     */
    [[nodiscard]] uint8_t metricCheck(const world::World* world, const world::Field* field) const noexcept override;

    /**
     * @brief Retrieves the name of the evaluation
     * @return A string containing the name of the evaluation
     */
    const char* getName() override { return "WeHaveMajority"; }
};
}  // namespace rtt::ai::stp::evaluation

#endif  // RTT_WEHAVEMAJORITYGLOBALEVALUATION_H

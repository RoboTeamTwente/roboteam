//
// Created by Luuk and Jorn on 11-12-2023
//

#ifndef RTT_WEWANTTOATTACKGLOBALEVALUATION_H
#define RTT_WEWANTTOATTACKGLOBALEVALUATION_H

#include "stp/evaluations/BaseEvaluation.h"

namespace rtt::ai::stp::evaluation {
/**
 * @brief Class that evaluates if we want to attack, based on ball position with regards to our goal
 */
class WeWantToAttackGlobalEvaluation : public BaseEvaluation {
   public:
    /**
     * @brief Calculates the score for whether we want to attack
     * @param world The current world
     * @param field The current field
     * @return The score of we want to attack
     */
    [[nodiscard]] uint8_t metricCheck(const world::World* world, const Field* field) const noexcept override;

    /**
     * @brief Retrieves the name of the evaluation
     * @return A string containing the name of the evaluation
     */
    const char* getName() override { return "WeWantToAttack"; }
};
}  // namespace rtt::ai::stp::evaluation

#endif  // RTT_WEWANTTOATTACKGLOBALEVALUATION_H

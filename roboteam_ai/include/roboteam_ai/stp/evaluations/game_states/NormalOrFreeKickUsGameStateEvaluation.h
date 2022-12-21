//
// Created by ratoone on 15-05-20.
//

#ifndef RTT_NORMALORFREEKICKUSGAMESTATEEVALUATION_H
#define RTT_NORMALORFREEKICKUSGAMESTATEEVALUATION_H

#include "stp/evaluations/BaseEvaluation.h"

namespace rtt::ai::stp::evaluation {
/**
 * @brief Class that evaluates the normal or free kick us game state
 */
class NormalOrFreeKickUsGameStateEvaluation : public BaseEvaluation {
    /**
     * @brief Calculates the score for the normal or free kick us game state
     * @param world The current world
     * @param field The current field
     * @return The score of the normal or free kick us game state
     */
    [[nodiscard]] uint8_t metricCheck(const world::World* world, const world::Field* field) const noexcept override;

    /**
     * @brief Retrieves the name of the evaluation
     * @return A string containing the name of the evaluation
     */
    const char* getName() override { return "gs::NormalOrFreeKickUs"; }
};
}  // namespace rtt::ai::stp::evaluation

#endif  // RTT_NORMALORFREEKICKUSGAMESTATEEVALUATION_H

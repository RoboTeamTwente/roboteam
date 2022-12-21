//
// Created by jordi on 28-04-20.
//

#ifndef RTT_KICKOFFTHEMGAMESTATEEVALUATION_H
#define RTT_KICKOFFTHEMGAMESTATEEVALUATION_H

#include "stp/evaluations/BaseEvaluation.h"

namespace rtt::ai::stp::evaluation {
/**
 * @brief Class that evaluates the kick off them game state
 */
class KickOffThemGameStateEvaluation : public BaseEvaluation {
   public:
    /**
     * @brief Calculates the score for the kick off them game state
     * @param world The current world
     * @param field The current field
     * @return The score of the kick off them game state
     */
    [[nodiscard]] uint8_t metricCheck(const world::World* world, const world::Field* field) const noexcept override;

    /**
     * @brief Retrieves the name of the evaluation
     * @return A string containing the name of the evaluation
     */
    const char* getName() override { return "gs::KickOffThem"; }
};
}  // namespace rtt::ai::stp::evaluation

#endif  // RTT_KICKOFFTHEMGAMESTATEEVALUATION_H

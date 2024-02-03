//
// Created by timovdk on 4/14/20.
//

#ifndef RTT_BALLPLACEMENTUSDIRECTGAMESTATEEVALUATION_H
#define RTT_BALLPLACEMENTUSDIRECTGAMESTATEEVALUATION_H

#include "stp/evaluations/BaseEvaluation.h"

namespace rtt::ai::stp::evaluation {
/**
 * @brief Class that evaluates the ball placement us game state when the next command is direct free kick
 */
class BallPlacementUsDirectGameStateEvaluation : public BaseEvaluation {
   public:
    /**
     * @brief Calculates the score for the ball placement us game state when the next command is direct free kick
     * @param world The current world
     * @param field The current field
     * @return The score of the ball placement direct us game state
     */
    [[nodiscard]] uint8_t metricCheck(const world::World* world, const Field* field) const noexcept override;

    /**
     * @brief Retrieves the name of the evaluation
     * @return A string containing the name of the evaluation
     */
    const char* getName() override { return "gs::BallPlacementUsDirect"; }
};
}  // namespace rtt::ai::stp::evaluation

#endif  // RTT_BALLPLACEMENTUSDIRECTGAMESTATEEVALUATION_H

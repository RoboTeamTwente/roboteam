//
// Created by timovdk on 5/27/20.
//

#ifndef RTT_NOGOALVISIONFROMBALLGLOBALEVALUATION_H
#define RTT_NOGOALVISIONFROMBALLGLOBALEVALUATION_H

#include <NFParam/Param.h>

#include "stp/evaluations/BaseEvaluation.h"

namespace rtt::ai::stp::evaluation {
/**
 * @brief Class that evaluates no goal vision from ball
 */
class NoGoalVisionFromBallGlobalEvaluation : public BaseEvaluation {
   public:
    /**
     * @brief Constructor for the NoGoalVisionFromBallGlobalEvaluation class
     */
    NoGoalVisionFromBallGlobalEvaluation() noexcept;
    
    /**
     * @brief Calculates the score for no goal vision from ball
     * @param world The current world
     * @param field The current field
     * @return The score of no goal vision from ball
     */
    [[nodiscard]] uint8_t metricCheck(const world::World* world, const world::Field* field) const noexcept override;

    /**
     * @brief Retrieves the name of the evaluation
     * @return A string containing the name of the evaluation
     */
    const char* getName() override { return "NoGoalVisionFromBall"; }

   private:
    /**
     * @brief Calculates the actual metric value using the piecewise linear function member
     * @param x the x of the function
     * @return metric value between 0-255
     */
    [[nodiscard]] uint8_t calculateMetric(const double& x) const noexcept;
    
    std::unique_ptr<nativeformat::param::Param> piecewiseLinearFunction; /**< Unique pointer to the piecewise linear function that calculates the fuzzy value */
};
}  // namespace rtt::ai::stp::evaluation

#endif  // RTT_NOGOALVISIONFROMBALLGLOBALEVALUATION_H

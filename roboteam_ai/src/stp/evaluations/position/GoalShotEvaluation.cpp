#include "stp/evaluations/position/GoalShotEvaluation.h"

#include <algorithm>
#include <cmath>

namespace rtt::ai::stp::evaluation {
uint8_t GoalShotEvaluation::metricCheck(double vis, double goalAngle, double normalizedDistanceToGoal) noexcept {
    auto evalScore = 0.5 * std::pow(goalAngle, 1.0 / 2.0) * vis + 0.5 * (1 - normalizedDistanceToGoal);
    return std::clamp(static_cast<int>(evalScore * 255), 0, 255);
}
}  // namespace rtt::ai::stp::evaluation
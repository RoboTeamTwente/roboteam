#ifndef RTT_OPENNESSEVALUATION_H
#define RTT_OPENNESSEVALUATION_H

#include <cstdint>
#include <vector>

namespace rtt::ai::stp::evaluation {
/**
 * @brief Class that evaluates the Openness score
 */
class OpennessEvaluation {
   public:
    /**
     * @brief Score openness of position based on enemy distances to point (quadratically correlated)
     * @param enemyDistances Vector of all enemy distances from the point
     * @param radius The max distance that enemy robots will affect a positions openess score
     * @return uint8_t score
     */
    [[nodiscard]] static uint8_t metricCheck(std::vector<double>& enemyDistances, double radius) noexcept;
};
}  // namespace rtt::ai::stp::evaluation
#endif  // RTT_OPENNESSEVALUATION_H

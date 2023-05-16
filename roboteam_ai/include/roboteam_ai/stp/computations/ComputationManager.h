//
// Created by alexander on 02-12-21.
//

#ifndef RTT_COMPUTATIONMANAGER_H
#define RTT_COMPUTATIONMANAGER_H

#include "roboteam_utils/Vector2.h"
#include "stp/constants/GeneralizationConstants.h"

namespace rtt::ai::stp {

/**
 * @brief Class that stores computation results that can be re-used in the same tick to prevent identical computations happening within a tick
 */
class ComputationManager {
   public:
    /**
     * @brief Clear computation results that are used to avoid computing the same thing more than once in a tick. This function should be called after every tick
     */
    static void clearStoredComputations() {
        calculatedScores.clear();
        calculatedWallPositions.clear();
    }

    inline static std::unordered_map<Vector2, gen::PositionScores> calculatedScores{}; /**< vector of calculated position scores */
    inline static std::vector<Vector2> calculatedWallPositions{}; /**< vector of determined wall positions */
};
}  // namespace rtt::ai::stp

#endif  // RTT_COMPUTATIONMANAGER_H

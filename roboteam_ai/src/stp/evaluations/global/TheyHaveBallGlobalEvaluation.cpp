#include "stp/evaluations/global/TheyHaveBallGlobalEvaluation.h"

namespace rtt::ai::stp::evaluation {
uint8_t TheyHaveBallGlobalEvaluation::metricCheck(const world::World* world, const Field*) const noexcept {
    auto& them = world->getWorld()->getThem();

    // If there are no bots, they don't have ball
    if (them.empty()) {
        return constants::FUZZY_FALSE;
    }
    return std::any_of(them.begin(), them.end(), [](auto& robot) { return robot->hasBall(); }) ? constants::FUZZY_TRUE : constants::FUZZY_FALSE;
}
}  // namespace rtt::ai::stp::evaluation

#include "stp/evaluations/global/TheyDoNotHaveBallGlobalEvaluation.h"

namespace rtt::ai::stp::evaluation {
uint8_t TheyDoNotHaveBallGlobalEvaluation::metricCheck(const world::World* world, const Field*) const noexcept {
    auto& them = world->getWorld()->getThem();

    // If there are no bots, they don't have ball
    if (them.empty()) {
        return stp::control_constants::FUZZY_TRUE;
    }
    return std::any_of(them.begin(), them.end(), [](auto& robot) { return robot->hasBall(); }) ? stp::control_constants::FUZZY_FALSE : stp::control_constants::FUZZY_TRUE;
}
}  // namespace rtt::ai::stp::evaluation

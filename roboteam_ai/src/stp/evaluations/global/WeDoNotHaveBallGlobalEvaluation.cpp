//
// Created by Luuk and Jorn on 12-12-23.
//

#include "stp/evaluations/global/WeDoNotHaveBallGlobalEvaluation.h"

namespace rtt::ai::stp::evaluation {
uint8_t WeDoNotHaveBallGlobalEvaluation::metricCheck(const world::World* world, const Field* field) const noexcept {
    auto& us = world->getWorld()->getUs();

    // If there are no bots, we don't have ball
    if (us.empty()) {
        return stp::control_constants::FUZZY_FALSE;
    }
    return std::any_of(us.begin(), us.end(), [](auto& robot) { return robot->hasBall(); }) ? stp::control_constants::FUZZY_FALSE : stp::control_constants::FUZZY_TRUE;
}
}  // namespace rtt::ai::stp::evaluation

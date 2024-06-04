#include "stp/evaluations/global/WeWillHaveBallGlobalEvaluation.h"

#include <stp/computations/InterceptionComputations.h>

namespace rtt::ai::stp::evaluation {
uint8_t WeWillHaveBallGlobalEvaluation::metricCheck(const world::World* world, const Field*) const noexcept {
    auto& us = world->getWorld()->getUs();
    auto& them = world->getWorld()->getThem();

    // If the opponent has no robot, we will get the ball
    if (them.empty()) return stp::control_constants::FUZZY_TRUE;

    // If we have no bots, we will not get the ball
    if (us.empty()) return stp::control_constants::FUZZY_FALSE;

    // If any of our robots has the ball, we will get the ball
    if (std::any_of(us.begin(), us.end(), [](auto& robot) { return robot->hasBall(); })) return stp::control_constants::FUZZY_TRUE;

    // If any of their robot has the ball has the ball, we will not get the ball
    if (std::any_of(them.begin(), them.end(), [](auto& robot) { return robot->hasBall(); })) return stp::control_constants::FUZZY_FALSE;

    // If we can intercept the ball, we will get the ball
    auto interceptionInfo = InterceptionComputations::calculateInterceptionInfo(us, world);
    if (interceptionInfo.isInterceptable) return stp::control_constants::FUZZY_TRUE;

    return stp::control_constants::FUZZY_FALSE;
}
}  // namespace rtt::ai::stp::evaluation
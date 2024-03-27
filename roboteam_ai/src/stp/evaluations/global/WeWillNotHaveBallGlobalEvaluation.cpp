#include "stp/evaluations/global/WeWillNotHaveBallGlobalEvaluation.h"

namespace rtt::ai::stp::evaluation {
uint8_t WeWillNotHaveBallGlobalEvaluation::metricCheck(const world::World* world, const Field*) const noexcept {
    auto& us = world->getWorld()->getUs();

    // If there are no bots, we will not get the ball
    if (us.empty()) {
        return stp::control_constants::FUZZY_FALSE;
    }
    // If our robots don't stand near the trajectory of the ball, we will most likely not get the ball
    auto ballPosition = world->getWorld()->getBall()->get()->position;
    auto endBallPosition = world->getWorld()->getBall()->get()->expectedEndPosition;
    return std::any_of(us.begin(), us.end(), [&](auto& robot) { return (LineSegment(ballPosition, endBallPosition).distanceToLine(robot->getPos()) < 0.3 || robot->hasBall()); })
               ? stp::control_constants::FUZZY_FALSE
               : stp::control_constants::FUZZY_TRUE;
}
}  // namespace rtt::ai::stp::evaluation
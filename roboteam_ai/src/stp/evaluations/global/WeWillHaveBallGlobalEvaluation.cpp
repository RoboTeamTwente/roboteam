#include "stp/evaluations/global/WeWillHaveBallGlobalEvaluation.h"

namespace rtt::ai::stp::evaluation {
uint8_t WeWillHaveBallGlobalEvaluation::metricCheck(const world::World* world, const Field*) const noexcept {
    auto& us = world->getWorld()->getUs();
    auto theirRobotCloseToBall = (world->getWorld()->getRobotClosestToBall(world::them))->get();

    // If there are no bots, we will not get the ball
    if (us.empty()) {
        return stp::control_constants::FUZZY_FALSE;
    }
    // If our robots don't stand near the trajectory of the ball, we will most likely not get the ball
    auto ballPosition = world->getWorld()->getBall()->get()->position;
    auto endBallPosition = world->getWorld()->getBall()->get()->expectedEndPosition;
    return std::any_of(us.begin(), us.end(),
                       [&](auto& robot) {
                           return (!theirRobotCloseToBall->hasBall() && (LineSegment(ballPosition, endBallPosition).distanceToLine(robot->getPos()) < 0.25 || robot->hasBall()));
                       })
               ? stp::control_constants::FUZZY_TRUE
               : stp::control_constants::FUZZY_FALSE;
}
}  // namespace rtt::ai::stp::evaluation
//
// Created by Luuk and Jorn on 12-12-2023
//

#include "stp/evaluations/global/WeWantToDefendGlobalEvaluation.h"

namespace rtt::ai::stp::evaluation {
uint8_t WeWantToDefendGlobalEvaluation::metricCheck(const world::World* world, const Field* field) const noexcept {
    auto theirRobots = world->getWorld()->getThem();
    auto ourRobots = world->getWorld()->getUs();
    auto ballPosition = world->getWorld()->getBall()->get()->position;
    auto ballVelocity = world->getWorld()->getBall()->get()->velocity.length();

    // If they don't have bots, we want to attack
    if (theirRobots.empty()) {
        return stp::control_constants::FUZZY_FALSE;
    }
    // If we have the ball, we want to attack
    if (std::any_of(ourRobots.begin(), ourRobots.end(), [](auto& robot) { return robot->hasBall(); })) {
        return stp::control_constants::FUZZY_FALSE;
    }
    // If they have the ball, we don't want to attack
    if (std::any_of(theirRobots.begin(), theirRobots.end(), [](auto& robot) { return robot->hasBall(); })) {
        return stp::control_constants::FUZZY_TRUE;
    }

    double distanceFromBallToCenterOfOurDefenseArea = (field->leftDefenseArea.rightLine().center() - ballPosition - ballVelocity * control_constants::DEALER_SPEED_FACTOR).length();
    if (distanceFromBallToCenterOfOurDefenseArea > field->playArea.width() * stp::control_constants::WE_WANT_TO_ATTACK_THRESHOLD) return stp::control_constants::FUZZY_FALSE;
    return stp::control_constants::FUZZY_TRUE;
}
}
   // namespace rtt::ai::stp::evaluation

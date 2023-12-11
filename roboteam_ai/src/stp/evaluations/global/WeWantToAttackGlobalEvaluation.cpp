//
// Created by Luuk and Jorn on 11-12-2023
//

#include "stp/evaluations/global/WewanttoattackGlobalEvaluation.h"

namespace rtt::ai::stp::evaluation {
uint8_t WeWantToAttackGlobalEvaluation::metricCheck(const world::World* world, const Field* field) const noexcept {
    auto& them = world->getWorld()->getThem();
    auto& us = world->getWorld()->getUs();
    auto& ball = world->getWorld()->getBall();

    // If they don't have bots, we want to attack
    if (them.empty()) {
        return stp::control_constants::FUZZY_TRUE;
    }
    // If we have the ball, we want to attack
    if std::any_of(us.begin(), us.end(), [](auto& robot) { return robot->hasBall(); }) {
        return stp::control_constants::FUZZY_TRUE;
    }
    // If they have the ball, we don't want to attack
    if std::any_of(them.begin(), them.end(), [](auto& robot) { return robot->hasBall(); }) {
        return stp::control_constants::FUZZY_FALSE;
    }
    // If the ball is in our defense area, we don't want to attack
    if (field->getOurDefenseArea().contains(ball->getPos())) {
        return stp::control_constants::FUZZY_FALSE;
    }
    auto ballToOurGoalScore = distanceFromPointToLine(field.leftGoalArea.bottomLeftCorner, field.leftGoalArea.topLeftCorner, ball->getPos() + ball->getVel());
    auto theirDistanceToBall = world->getWorld()->getRobotClosestToBall(them)->get()->getPos().dist(ball->getPos()) + world->getWorld()->getRobotClosestToBall(them)->get()->getVel().length();
    auto ourDistanceToBall = world->getWorld()->getRobotClosestToBall(us)->get()->getPos().dist(ball->getPos()) + world->getWorld()->getRobotClosestToBall(us)->get()->getVel().length();
}
}  // namespace rtt::ai::stp::evaluation

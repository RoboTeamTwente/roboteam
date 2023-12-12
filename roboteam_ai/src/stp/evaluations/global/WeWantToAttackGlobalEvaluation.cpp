//
// Created by Luuk and Jorn on 11-12-2023
//

#include "stp/evaluations/global/WeWantToAttackGlobalEvaluation.h"

namespace rtt::ai::stp::evaluation {
uint8_t WeWantToAttackGlobalEvaluation::metricCheck(const world::World* world, const Field* field) const noexcept {
    auto theirRobots = world->getWorld()->getThem();
    auto ourRobots = world->getWorld()->getUs();
    auto ballPosition = world->getWorld()->getBall()->get()->position;
    auto ballVelocity = world->getWorld()->getBall()->get()->velocity.length();

    // If they don't have bots, we want to attack
    if (theirRobots.empty()) {
        return stp::control_constants::FUZZY_TRUE;
    }
    // If we have the ball, we want to attack
    if (std::any_of(ourRobots.begin(), ourRobots.end(), [](auto& robot) { return robot->hasBall(); })) {
        return stp::control_constants::FUZZY_TRUE;
    }
    // If they have the ball, we don't want to attack
    if (std::any_of(theirRobots.begin(), theirRobots.end(), [](auto& robot) { return robot->hasBall(); })) {
        return stp::control_constants::FUZZY_FALSE;
    }
    // If the ball is in our defense area, we don't want to attack
    if (field->leftDefenseArea.contains(ballPosition)) {
        return stp::control_constants::FUZZY_FALSE;
    }

    // int bestBotThemDistance = 0;
    // int bestBotUsDistance = 0;

    // //Magic numbers to tune
    // auto ballVelocityFactor = 0.5;
    // auto robotVelocityFactor = 0.5;
    // int score_threshold = 160;
    // auto distanceToGoalWeight = 0.6;
    // auto distanceTheirBotWeight = 0.9;
    // auto distanceOurBotWeight = 0.85;

    // auto ballToOurGoalScore = distanceFromPointToLine(field.leftGoalArea.bottomLeft() field.leftGoalArea.topLeft(), ballPosition + ballVelocity);

    // for (const auto &theirRobot : theirRobots) {
    //     auto distanceBotToBall = (theirRobot->getPos() + theirRobot->getVel().length() * robotVelocityFactor - ballPosition + ballVelocity * ballVelocityFactor).length;
    //     if (distanceBotToBall < bestBotThemDistance) {
    //         auto bestBotThemDistance = distanceBotToBall;
    //     }
    // }
    // for (const auto &ourRobot : ourRobots) {
    //     auto distanceBotToBall = (ourRobot->getPos() + ourRobot->getVel().length() * robotVelocityFactor - ballPosition + ballVelocity * ballVelocityFactor).length;
    //     if (distanceBotToBall < bestBotUsDistance) {
    //         auto bestBotUsDistance = distanceBotToBall;
    //     }
    // }
    // std::cout<<ballToOurGoalScore<<bestBotThemDistance<<bestBotUsDistance<<std::endl;

    // totalScore = distanceOurBotWeight * bestBotUsDistance - distanceTheirBotWeight * bestBotThemDistance - distanceToGoalWeight * ballToOurGoalsScore;
    // if (totalScore < score_threshold) return stp::control_constants::FUZZY_FALSE;
    // return stp::control_constants::FUZZY_TRUE;
}
}
   // namespace rtt::ai::stp::evaluation

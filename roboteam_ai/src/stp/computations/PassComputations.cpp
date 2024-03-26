//
// Created by maxl on 11-02-21.
//

#include <roboteam_utils/Grid.h>
#include <stp/computations/PassComputations.h>
#include <stp/computations/PositionComputations.h>
#include <stp/computations/PositionScoring.h>

#include "control/ControlUtils.h"
#include "gui/Out.h"
#include "roboteam_utils/Tube.h"
#include "stp/constants/ControlConstants.h"

namespace rtt::ai::stp::computations {

PassInfo PassComputations::calculatePass(gen::ScoreProfile profile, const rtt::world::World* world, const Field& field, bool keeperCanPass) {
    PassInfo passInfo;  // Struct used to store the information needed to execute the pass
    if (world->getWorld()->getUs().size() < (keeperCanPass ? 2 : 3)) {
        return passInfo;
    }

    auto us = world->getWorld()->getUs();
    auto ballLocation = world->getWorld()->getBall()->get()->position;

    // Find which robot is keeper (bot closest to goal if there was not a keeper yet), store its id, and erase from us
    passInfo.keeperId = getKeeperId(us, world, field);
    if (!keeperCanPass) std::erase_if(us, [passInfo](auto& bot) { return bot->getId() == passInfo.keeperId; });

    // Remove cardId from us
    auto cardId = GameStateManager::getCurrentGameState().cardId;
    std::erase_if(us, [cardId](auto& bot) { return bot->getId() == cardId; });

    // Find which robot should be the passer, store its id and location, and erase from us
    passInfo.passerId = getPasserId(ballLocation, us, world);
    auto passerIt = std::find_if(us.begin(), us.end(), [passInfo](auto& bot) { return bot->getId() == passInfo.passerId; });

    Vector2 passerLocation;
    Vector2 passerVelocity;
    // there should always be a valid passer, since we know there are >2 robots (or 2 robots where the keeper can pass/receive), but check just in case something goes wrong
    if (passerIt != us.end()) {
        passerLocation = passerIt->get()->getPos();
        passerVelocity = passerIt->get()->getVel();
        us.erase(passerIt);
    } else {
        // If we could not find a passer, we return an empty passInfo
        return {};
    }

    // This is a vector with the locations of all robots that could act as a receiver (ie all robots except the keeper and the passer)
    std::vector<Vector2> possibleReceiverLocations;
    std::vector<Vector2> possibleReceiverVelocities;
    // Add all robots that can also kick (nice for kicking at goal or passing further)
    for (const auto& robot : us) {
        if (Constants::ROBOT_HAS_KICKER(robot->getId())) possibleReceiverLocations.push_back(robot->getPos());
        if (Constants::ROBOT_HAS_KICKER(robot->getId())) possibleReceiverVelocities.push_back(robot->getPos());
    }
    // If there are no other robots that can kick, add every other robots
    if (possibleReceiverLocations.empty()) {
        possibleReceiverLocations.reserve(us.size());
        for (const auto& robot : us) {
            possibleReceiverLocations.push_back(robot->getPos());
        }
    }
    if (possibleReceiverVelocities.empty()) {
        possibleReceiverVelocities.reserve(us.size());
        for (const auto& robot : us) {
            possibleReceiverVelocities.push_back(robot->getPos());
        }
    }

    // Now find out the best pass location and corresponding info
    auto possiblePassLocationsVector = getPassGrid(field).getPoints();
    int numberOfPoints = 0;
    for (auto& pointVector : possiblePassLocationsVector) {
        for (auto& point : pointVector) {
            numberOfPoints++;
            std::array<rtt::Vector2, 1> pointToPassTo = {point};
            if (pointIsValidPassLocation(point, ballLocation, possibleReceiverLocations, possibleReceiverVelocities, passerLocation, passerVelocity, field, world)) {
                gen::ScoredPosition scoredPosition = PositionScoring::scorePosition(point, profile, field, world);
                rtt::ai::gui::Out::draw(
                    {
                        .label = "pass_location" + std::to_string(numberOfPoints),
                        .color = scoredPosition.score >= 245 ? proto::Drawing::GREEN : proto::Drawing::MAGENTA,
                        .method = proto::Drawing::PLUSES,
                        .category = proto::Drawing::DEBUG,
                        .forRobotId = passInfo.passerId,
                        .size = static_cast<int>(pow(scoredPosition.score / 40, 2) + 4),
                        .thickness = static_cast<int>(pow(scoredPosition.score / 80, 2) + 3),
                    },
                    pointToPassTo);
                if (scoredPosition.score > passInfo.passScore) {
                    passInfo.passScore = scoredPosition.score;
                    passInfo.passLocation = scoredPosition.position;
                    passInfo.receiverId = world->getWorld()->getRobotClosestToPoint(passInfo.passLocation, us).value()->getId();
                }
            }
        }
    }
    if (passInfo.passScore == 0) {
        // If no good pass is found, pass to the robot furthest in the field
        auto furthestRobotIt = std::max_element(possibleReceiverLocations.begin(), possibleReceiverLocations.end(), [](auto& p1, auto& p2) { return p1.x > p2.x; });
        // We should always be able to find a furthest robot, this check avoids the AI crashing in case something does go wrong due to changes/bugs
        passInfo.passLocation = (furthestRobotIt != possibleReceiverLocations.end()) ? *furthestRobotIt : Vector2();
    }

    return passInfo;
}

Grid PassComputations::getPassGrid(const Field& field) {
    double gridHeight = field.playArea.height();
    double gridWidth = field.playArea.width();
    int numPoints = 9;
    return Grid(-gridWidth / 2, -gridHeight / 2, gridWidth, gridHeight, numPoints, numPoints);  // 81 points spread over the whole field
}

bool PassComputations::pointIsValidPassLocation(Vector2 point, Vector2 ballLocation, const std::vector<Vector2>& possibleReceiverLocations,
                                                const std::vector<Vector2>& possibleReceiverVelocities, Vector2 passerLocation, Vector2 passerVelocity, const Field& field,
                                                const world::World* world) {
    constexpr double MINIMUM_PASS_DISTANCE = 2.0;  // This can be dribbled instead of passed
    if (point.dist(ballLocation) < MINIMUM_PASS_DISTANCE) return false;
    constexpr double MINIMUM_LINE_OF_SIGHT = 10.0;  // The minimum LoS to be a valid pass, otherwise, the pass will go into an enemy robot
    if (PositionScoring::scorePosition(point, gen::LineOfSight, field, world).score < MINIMUM_LINE_OF_SIGHT) return false;
    if (!FieldComputations::pointIsValidPosition(field, point)) return false;
    // Pass is valid if the above conditions are met and there is a robot whose travel time is smaller than the balls travel time (i.e. the robot can actually receive the ball)
    auto ballTravelTime = calculateBallTravelTime(ballLocation, passerLocation, passerVelocity, point);
    for (int i = 0; i < possibleReceiverLocations.size(); i++) {
        if (calculateRobotTravelTime(possibleReceiverLocations[i], possibleReceiverVelocities[i], point) < ballTravelTime) return true;
    }
    return false;
}

int PassComputations::getPasserId(Vector2 ballLocation, const std::vector<world::view::RobotView>& ourRobots, const world::World* world) {
    int bestPasserId = -1;
    auto field = world->getField().value();

    auto possiblePassers = ourRobots;
    // Remove robots that cannot kick
    std::erase_if(possiblePassers, [](const world::view::RobotView& rbv) { return !Constants::ROBOT_HAS_KICKER(rbv->getId()); });

    // If there is no robot that can kick, return -1
    if (possiblePassers.empty()) return bestPasserId;
    // If there is at least one, pick the closest one to the ball as best passer
    auto closestPasserId = PositionComputations::calculateInterceptionInfo(field, world, -1).interceptId;
    if (closestPasserId != -1) bestPasserId = closestPasserId;

    // Remove robots that cannot detect the ball themselves (so no ballSensor or dribblerEncoder)
    std::erase_if(possiblePassers, [](const world::view::RobotView& rbv) { return !Constants::ROBOT_HAS_WORKING_DRIBBLER_ENCODER(rbv->getId()); });

    // If no robot can detect the ball, the previous closest robot that can only kick is the best one
    if (possiblePassers.empty()) return bestPasserId;
    // But if there is one, the current best passer will be the closest one
    closestPasserId = PositionComputations::calculateInterceptionInfo(field, world, -1).interceptId;
    if (closestPasserId != -1) bestPasserId = closestPasserId;

    return bestPasserId;
}

int PassComputations::getKeeperId(const std::vector<world::view::RobotView>& possibleRobots, const world::World* world, const Field& field) {
    auto keeperId = GameStateManager::getCurrentGameState().keeperId;
    auto keeperIt = std::find_if(possibleRobots.begin(), possibleRobots.end(), [keeperId](const auto& bot) { return bot->getId() == keeperId; });
    if (keeperIt != possibleRobots.end()) {
        return (*keeperIt)->getId();
    }
    auto keeper = world->getWorld()->getRobotClosestToPoint(field.leftGoalArea.rightLine().center(), possibleRobots);
    if (keeper) {
        return keeper->get()->getId();
    }
    return -1;  // Should never reach this point unless there are no robots
}

double PassComputations::calculateRobotTravelTime(Vector2 robotPosition, Vector2 robotVelocity, Vector2 targetPosition) {
    return Trajectory2D(robotPosition, robotVelocity, targetPosition, control::ControlUtils::getMaxVelocity(false), ai::Constants::MAX_ACC_UPPER()).getTotalTime() * 1.1;
}

double PassComputations::calculateBallTravelTime(Vector2 ballPosition, Vector2 passerLocation, Vector2 passerVelocity, Vector2 targetPosition) {
    auto travelTime = calculateRobotTravelTime(passerLocation, passerVelocity, ballPosition - (passerLocation - ballPosition).stretchToLength(control_constants::ROBOT_RADIUS));
    auto rotateTime = (ballPosition - passerLocation).toAngle().shortestAngleDiff(targetPosition - ballPosition) / (M_PI);
    double ballSpeed = control::ControlUtils::determineKickForce(ballPosition.dist(targetPosition), ShotType::PASS);
    auto ballTime = ballPosition.dist(targetPosition) / ballSpeed;
    return travelTime + rotateTime + ballTime;
}

}  // namespace rtt::ai::stp::computations
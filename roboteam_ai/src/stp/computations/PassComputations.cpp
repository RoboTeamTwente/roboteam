#include <roboteam_utils/Grid.h>
#include <stp/computations/InterceptionComputations.h>
#include <stp/computations/PassComputations.h>
#include <stp/computations/PositionComputations.h>
#include <stp/computations/PositionScoring.h>

#include "control/ControlUtils.h"
#include "gui/Out.h"
#include "roboteam_utils/Tube.h"
#include "utilities/Constants.h"

namespace rtt::ai::stp::computations {

PassInfo PassComputations::calculatePass(gen::ScoreProfile profile, const rtt::world::World* world, const Field& field, bool keeperMustPass) {
    PassInfo passInfo;  // Struct used to store the information needed to execute the pass
    if (world->getWorld()->getUs().size() < (keeperMustPass ? 2 : 3)) {
        return passInfo;
    }

    auto us = world->getWorld()->getUs();

    // Find which robot is keeper (bot closest to goal if there was not a keeper yet), store its id, and erase from us to avoid the desired keeper to be the passer
    passInfo.keeperId = InterceptionComputations::getKeeperId(us, world);
    if (!keeperMustPass) std::erase_if(us, [passInfo](auto& bot) { return bot->getId() == passInfo.keeperId; });

    // Remove cardId from us
    auto cardId = GameStateManager::getCurrentGameState().cardId;
    std::erase_if(us, [cardId](auto& bot) { return bot->getId() == cardId; });

    // Find which robot should be the passer, store its id and location, and erase from us
    InterceptionInfo interceptionInfo;
    auto keeper = world->getWorld()->getRobotForId(passInfo.keeperId, true);
    if (keeperMustPass && keeper) {
        interceptionInfo = InterceptionComputations::calculateInterceptionInfoForKickingRobots({keeper.value()}, world);
    } else {
        interceptionInfo = InterceptionComputations::calculateInterceptionInfoForKickingRobots(us, world);
    }
    passInfo.passerId = interceptionInfo.interceptId;
    passInfo.passLocation = interceptionInfo.interceptLocation;
    auto passerIt = std::find_if(us.begin(), us.end(), [passInfo](auto& bot) { return bot->getId() == passInfo.passerId; });

    Vector2 passerLocation;
    Vector2 passerVelocity;
    int passerId;
    // there should always be a valid passer, since we know there are >2 robots (or 2 robots where the keeper can pass/receive), but check just in case something goes wrong
    if (passerIt != us.end()) {
        passerLocation = passerIt->get()->getPos();
        passerVelocity = passerIt->get()->getVel();
        passerId = passerIt->get()->getId();
        us.erase(passerIt);
    } else {
        // If we could not find a passer, we return an empty passInfo
        return {};
    }

    // This is a vector with the locations of all robots that could act as a receiver (ie all robots except the keeper and the passer)
    std::vector<Vector2> possibleReceiverLocations;
    std::vector<Vector2> possibleReceiverVelocities;
    std::vector<int> possibleReceiverIds;
    for (const auto& robot : us) {
        if (constants::ROBOT_HAS_KICKER(robot->getId())) {
            possibleReceiverLocations.push_back(robot->getPos());
            possibleReceiverVelocities.push_back(robot->getVel());
            possibleReceiverIds.push_back(robot->getId());
        }
    }
    // If there are no other robots that can kick, add every other robots
    if (possibleReceiverLocations.empty()) {
        possibleReceiverLocations.reserve(us.size());
        possibleReceiverVelocities.reserve(us.size());
        possibleReceiverIds.reserve(us.size());
        for (const auto& robot : us) {
            possibleReceiverLocations.push_back(robot->getPos());
            possibleReceiverVelocities.push_back(robot->getVel());
            possibleReceiverIds.push_back(robot->getId());
        }
    }

    // Now find out the best pass location and corresponding info
    auto possibleReceiverLocationsVector = getPassGrid(field).getPoints();
    int numberOfPoints = 0;
    for (auto& pointVector : possibleReceiverLocationsVector) {
        for (auto& point : pointVector) {
            numberOfPoints++;
            std::array<rtt::Vector2, 1> pointToPassTo = {point};
            if (pointIsValidReceiverLocation(point, possibleReceiverLocations, possibleReceiverVelocities, possibleReceiverIds, passInfo.passLocation, passerLocation,
                                             passerVelocity, passerId, field, world)) {
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
                    passInfo.receiverLocation = scoredPosition.position;
                    passInfo.receiverId = world->getWorld()->getRobotClosestToPoint(passInfo.receiverLocation, us).value()->getId();
                }
            }
        }
    }
    if (passInfo.passScore == 0) {
        // If no good pass is found, pass to the robot furthest in the field
        auto furthestRobotIt = std::max_element(possibleReceiverLocations.begin(), possibleReceiverLocations.end(), [](const auto& p1, const auto& p2) { return p1.x > p2.x; });
        // We should always be able to find a furthest robot, this check avoids the AI crashing in case something does go wrong due to changes/bugs
        passInfo.receiverLocation = (furthestRobotIt != possibleReceiverLocations.end()) ? *furthestRobotIt : Vector2();
    }

    return passInfo;
}

Grid PassComputations::getPassGrid(const Field& field) {
    double gridHeight = field.playArea.height();
    double gridWidth = field.playArea.width();
    int numPoints = 9;
    return Grid(-gridWidth / 2, -gridHeight / 2, gridWidth, gridHeight, numPoints, numPoints);  // 81 points spread over the whole field
}

bool PassComputations::pointIsValidReceiverLocation(Vector2 point, const std::vector<Vector2>& possibleReceiverLocations, const std::vector<Vector2>& possibleReceiverVelocities,
                                                    const std::vector<int>& possibleReceiverIds, Vector2 passLocation, Vector2 passerLocation, Vector2 passerVelocity, int passerId,
                                                    const Field& field, const world::World* world) {
    constexpr double MINIMUM_PASS_DISTANCE = 2.0;  // This can be dribbled instead of passed
    if (point.dist(passLocation) < MINIMUM_PASS_DISTANCE) return false;
    constexpr double MINIMUM_LINE_OF_SIGHT = 10.0;  // The minimum LoS to be a valid pass, otherwise, the pass will go into an enemy robot
    if (PositionScoring::scorePosition(point, gen::LineOfSight, field, world).score < MINIMUM_LINE_OF_SIGHT) return false;
    AvoidObjects avoidObj;
    avoidObj.shouldAvoidOurDefenseArea = true;
    avoidObj.shouldAvoidTheirDefenseArea = true;
    avoidObj.shouldAvoidOutOfField = true;
    if (!FieldComputations::pointIsValidPosition(field, point, avoidObj)) return false;
    // Pass is valid if the above conditions are met and there is a robot whose travel time is smaller than the balls travel time (i.e. the robot can actually receive the ball)
    auto ballTravelTime = calculateBallTravelTime(passLocation, passerLocation, passerId, passerVelocity, point);
    for (std::vector<rtt::Vector2>::size_type i = 0; i < possibleReceiverLocations.size(); i++) {
        if (calculateRobotTravelTime(possibleReceiverLocations[i], possibleReceiverVelocities[i], possibleReceiverIds[i], point) < ballTravelTime) return true;
    }
    return false;
}

double PassComputations::calculateRobotTravelTime(Vector2 robotPosition, Vector2 robotVelocity, int robotId, Vector2 targetPosition) {
    return Trajectory2D(robotPosition, robotVelocity, targetPosition, control::ControlUtils::getMaxVelocity(false), ai::constants::MAX_ACC, ai::constants::MAX_JERK_DEFAULT,
                        robotId)
               .getTotalTime() *
           1.1;
}

double PassComputations::calculateBallTravelTime(Vector2 passLocation, Vector2 passerLocation, int passerId, Vector2 passerVelocity, Vector2 targetPosition) {
    auto travelTime = calculateRobotTravelTime(passerLocation, passerVelocity, passerId, passLocation - (passerLocation - passLocation).stretchToLength(constants::ROBOT_RADIUS));
    auto rotateTime = (passLocation - passerLocation).toAngle().shortestAngleDiff(targetPosition - passLocation) / (M_PI);
    double ballSpeed = control::ControlUtils::determineKickForce(passLocation.dist(targetPosition), ShotPower::PASS);
    auto ballTime = passLocation.dist(targetPosition) / ballSpeed;
    return travelTime + rotateTime + ballTime;
}

}  // namespace rtt::ai::stp::computations
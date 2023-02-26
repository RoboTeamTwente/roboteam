//
// Created by martin on 11-5-22.
//

#include "control/positionControl/CollisionDetector.h"

#include <span>

#include "control/positionControl/PositionControlUtils.h"

namespace rtt::ai::control {

bool CollisionDetector::doesCollideWithMovingObjects(const Vector2& position, int robotId, bool shouldAvoidBall, double time) const {
    int timeStep = PositionControlUtils::convertTimeToStep(time);
    if (timeStep >= timeline.size()) {
        return false;
    }

    const auto& obstacles = timeline[timeStep];
    return (shouldAvoidBall && isCollision(position, obstacles.ball.position, minBallDistance))
    || std::any_of(obstacles.robotsUs.cbegin(), obstacles.robotsUs.cend(), [&](auto& otherRobot) {
        return otherRobot.first != robotId && isCollision(position, otherRobot.second.position, PositionControlUtils::MIN_ROBOT_DISTANCE);
    })
    || std::any_of(obstacles.robotsThem.cbegin(), obstacles.robotsThem.cbegin(), [&](auto& otherRobot) {
        return isCollision(position, otherRobot.position, PositionControlUtils::MIN_ROBOT_DISTANCE);
    });
}

bool CollisionDetector::doesCollideWithField(const Vector2& position) const {
    // TODO: Restore pointIsInField method
    // return rtt::ai::FieldComputations::pointIsInField(field.value(), position, rtt::ai::Constants::ROBOT_RADIUS());
    return false;
}

bool CollisionDetector::doesCollideWithDefenseArea(const Vector2& position) const {
    return ourDefenseArea.contains(position) && !theirDefenseArea.contains(position);
}

bool CollisionDetector::isCollision(const Vector2& position, const Vector2& obstaclePos, double minDistance) {
    return (position - obstaclePos).length() < minDistance;
}

void CollisionDetector::updateTimeline(const std::vector<RobotView>& robots, const std::optional<BallView>& ball) {
    for (int i = 0; i < PositionControlUtils::COLLISION_DETECTOR_STEP_COUNT; i++) {
        const double time = PositionControlUtils::convertStepToTime(i);
        auto& positionsAtTime = timeline[i];
        positionsAtTime.robotsThem.clear();

        for (const auto& robot : robots) {
            if (robot->getTeam() == rtt::world::Team::them) {
                positionsAtTime.robotsThem.emplace_back(StateVector{robot->getPos() + robot->getVel() * time, robot->getVel()});
                continue;
            }

            if (!PositionControlUtils::isMoving(robot->getPos()) || i == 0) {
                positionsAtTime.robotsUs[robot->getId()] = StateVector{robot->getPos(), Vector2{0, 0}};
            }
        }

        positionsAtTime.ball = ball.has_value()
            ? StateVector{ball->get()->position + ball->get()->velocity * time, ball->get()->velocity}
            : StateVector{};
    }
}

void CollisionDetector::setMinBallDistance(double distance) { minBallDistance = distance; }

void CollisionDetector::updateDefenseAreas(const std::optional<rtt::Field>& field) {
    if (!field.has_value()) { return; }
    ourDefenseArea = rtt::ai::FieldComputations::getDefenseArea(field.value(), true, 0, 0);
    theirDefenseArea = rtt::ai::FieldComputations::getDefenseArea(field.value(), false, 0, 0);
}

void CollisionDetector::updateTimelineForOurRobot(std::span<const StateVector> path, const Vector2& currentPosition, int robotId) {
    // Sometimes robot did not reach the next step, thus we want to offset the collision by 1.
    // timeline[0] is filed with current position at the start of the tick
    int offset = !path.empty() && !PositionControlUtils::isTargetReached(path[0].position, currentPosition) ? 1 : 0;
    for (int i = offset; i < PositionControlUtils::COLLISION_DETECTOR_STEP_COUNT; i++) {
        timeline[i].robotsUs[robotId] = i < path.size() ? path[i] : StateVector{currentPosition, Vector2{0, 0}};
    }
}
}  // namespace rtt::ai::control

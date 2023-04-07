//
// Created by martin on 11-5-22.
//

#include "control/positionControl/CollisionDetector.h"

#include <span>

#include "control/positionControl/PositionControlUtils.h"

namespace rtt::ai::control {

bool CollisionDetector::doesCollideWithMovingObjects(const Vector2& position, int robotId, const stp::AvoidObjects& avoidObjects, int timeStep) const {
    if (timeStep >= static_cast<int>(timeline.size())) {
        return false;
    }

    const auto& obstacles = timeline[timeStep];
    return (avoidObjects.shouldAvoidBall && isCollision(position, obstacles.ball.position, avoidObjects.avoidBallDist)) ||
           (avoidObjects.shouldAvoidOurRobots && std::any_of(obstacles.robotsUs.cbegin(), obstacles.robotsUs.cend(),
                                                             [&](auto& otherRobot) {
                                                                 return otherRobot.first != robotId &&
                                                                        isCollision(position, otherRobot.second.position, PositionControlUtils::MIN_ROBOT_DISTANCE);
                                                             })) ||
           (avoidObjects.shouldAvoidTheirRobots && std::any_of(obstacles.robotsThem.cbegin(), obstacles.robotsThem.cend(), [&](auto& otherRobot) {
                return isCollision(position, otherRobot.position, PositionControlUtils::MIN_ROBOT_DISTANCE);
            }));
}

bool CollisionDetector::isCollision(const Vector2& position, const Vector2& obstaclePos, double minDistance) {
    double distance = (position - obstaclePos).length();
    return distance < minDistance;
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

        positionsAtTime.ball = ball.has_value() ? StateVector{ball->get()->position + ball->get()->velocity * time, ball->get()->velocity} : StateVector{};
    }
}

void CollisionDetector::setField(rtt::Field newField) {
    field = std::move(newField);
}

void CollisionDetector::updateTimelineForOurRobot(std::span<const StateVector> path, const Vector2& currentPosition, int robotId) {
    int pathLength = static_cast<int>(path.size());

    // Sometimes robot did not reach the next step, thus we want to offset the collision by 1.
    // timeline[0] is filed with current position at the start of the tick
    int offset = !path.empty() && !PositionControlUtils::isTargetReached(path[0].position, currentPosition) ? 1 : 0;
    for (int i = offset; i < PositionControlUtils::COLLISION_DETECTOR_STEP_COUNT && i < pathLength; i++) {
        timeline[i].robotsUs[robotId] = path[i];
    }

    // Fill the rest of the timeline with the last position
    const auto endState = path.empty() ? StateVector{currentPosition, Vector2{0, 0}} : StateVector{path[pathLength - 1].position, Vector2{0, 0}};

    for (int i = pathLength; i < PositionControlUtils::COLLISION_DETECTOR_STEP_COUNT; i++) {
        timeline[i].robotsUs[robotId] = endState;
    }
}
bool CollisionDetector::doesCollideWithStaticObjects(const Vector2& position, const stp::AvoidObjects& avoidObjects) const {
    // TODO: Restore pointIsInField method
    //     return rtt::ai::FieldComputations::pointIsInField(field.value(), position, rtt::ai::Constants::ROBOT_RADIUS());
    //    return theirDefenseArea.contains(position) || (avoidObjects.shouldAvoidDefenseArea && ourDefenseArea.contains(position));
    return field.rightDefenseArea.contains(position) || (avoidObjects.shouldAvoidDefenseArea && field.leftDefenseArea.contains(position)) ||
           (avoidObjects.shouldAvoidOutOfField && field.playArea.contains(position));
}
void CollisionDetector::drawTimeline() const {
    for (int i = 0; i < PositionControlUtils::COLLISION_DETECTOR_STEP_COUNT; i++) {
        const auto& obstacles = timeline[i];
        interface::Input::drawData(interface::Visual::PATHFINDING, {obstacles.ball.position}, Qt::red, interface::Drawing::CROSSES);

        for (const auto& robot : obstacles.robotsThem) {
            interface::Input::drawData(interface::Visual::PATHFINDING, {robot.position}, Qt::red, interface::Drawing::CROSSES);
        }
    }
}
}  // namespace rtt::ai::control

//
// Created by tijmen on 27-10-21.
//

#include "control/positionControl/PathTracking.h"

#include <stp/StpInfo.h>

#include <span>

#include "roboteam_utils/Print.h"

namespace rtt::ai::control {

std::pair<std::span<StateVector>, Position> PathTracking::trackPath(const StateVector& currentState, std::span<StateVector> remainingPath, std::pair<PID, PID> pidControllers, stp::PIDType pidType) const{
    if (remainingPath.empty()) {
        return {remainingPath, {0, 0, 0}};
    }

//    interface::Input::drawData(interface::Visual::PATHFINDING, {remainingPath}, Qt::red, -1, interface::Drawing::DOTS);

    auto lookAhead = std::min(remainingPath.size(), STEPS_AHEAD);
    const auto &currentTarget = std::next(remainingPath.begin(), lookAhead - 1);
    if (PositionControlUtils::isTargetReached(currentTarget->position, currentState.position)) {
        // Track the Nth point, or the last if the size is smaller than N; the untracked ones are discarded
        remainingPath = remainingPath.subspan(lookAhead);
    }

    const auto newPid = PositionControlUtils::getPIDValue(pidType);
    pidControllers.first.setPID(newPid);
    pidControllers.second.setPID(newPid);

    auto pidVelocity = Vector2 {
        pidControllers.first.getOutput(currentState.position.x, currentTarget->position.x),
        pidControllers.second.getOutput(currentState.position.y, currentTarget->position.y),
    }.stretchToLength(currentTarget->velocity.length());

    return {remainingPath, {pidVelocity.x, pidVelocity.y, (currentTarget->position - currentState.position).angle()}};
}

UpdatePath PathTracking::shouldUpdatePath(const PositionControlInput& input, std::span<StateVector> remainingPath) const {
    if (!remainingPath.empty()) {
        if (PositionControlUtils::isTargetChanged(input.targetPos, remainingPath.back().position)) return UPDATE_TARGET_CHANGED;
        if (PositionControlUtils::isTargetChanged(input.state.position, remainingPath.front().position)) return UPDATE_POSITION_CHANGED;
    }

    if (remainingPath.empty() && !PositionControlUtils::isTargetReached(input.targetPos, input.state.position)) return UPDATE_TARGET_REACHED;


//    auto collision = collisionDetector.getFirstCollision(remainingPath, robotId, avoidObjects);
//    if (collision.has_value() && remainingPath.size() > 2) return UPDATE_COLLISION_DETECTED;

    return DONT_UPDATE;
}

PathTracking::PathTracking(const CollisionDetector &collisionDetector) : collisionDetector(collisionDetector) {}

}  // namespace rtt::ai::control
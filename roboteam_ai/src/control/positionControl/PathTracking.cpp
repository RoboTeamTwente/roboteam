//
// Created by tijmen on 27-10-21.
//

#include "control/positionControl/PathTracking.h"

#include <stp/StpInfo.h>

#include <span>

#include "roboteam_utils/Print.h"

namespace rtt::ai::control {

std::pair<std::span<StateVector>, Position> PathTracking::trackPath(const StateVector& currentState, std::span<StateVector> remainingPath, std::pair<PID, PID> pidControllers,
                                                                    stp::PIDType pidType) const {
    if (remainingPath.empty()) {
        return {{}, {0, 0, 0}};
    }


    const auto destination = remainingPath.back().position;
    auto lookAhead = std::min(remainingPath.size(), STEPS_AHEAD);
    auto currentTarget = std::next(remainingPath.begin(), lookAhead - 1);
    if (PositionControlUtils::positionWithinTolerance(currentTarget->position, currentState.position) ||          // if we reached the target on the path, set the next point as target or
        (destination - currentState.position).length() < (destination - currentTarget->position).length())  // if we are closer to the target than the next point on the path
    {
        remainingPath = remainingPath.subspan(lookAhead);
        currentTarget = std::next(remainingPath.begin(), lookAhead - 1);
    }

    const auto newPid = PositionControlUtils::getPIDValue(pidType);
    pidControllers.first.setPID(newPid);
    pidControllers.second.setPID(newPid);

    auto pidVelocity =
        Vector2{
            pidControllers.first.getOutput(currentState.position.x, currentTarget->position.x),
            pidControllers.second.getOutput(currentState.position.y, currentTarget->position.y),
        }
            .stretchToLength(currentTarget->velocity.length());

    return {remainingPath, {pidVelocity.x, pidVelocity.y, (currentTarget->position - currentState.position).angle()}};
}

UpdatePath PathTracking::shouldUpdatePath(const PositionControlInput& input, std::span<StateVector> remainingPath) const {
    if (!remainingPath.empty()) {
        // Check if the target position has changed in the middle of path traversal
        if (!PositionControlUtils::positionWithinTolerance(input.targetPos, remainingPath.back().position)) return UPDATE_TARGET_CHANGED;

        // If the robots suddenly moves a lot from next position, we should update the path
        //        if (PositionControlUtils::positionWithinTolerance(input.state.position, remainingPath.front().position)) return UPDATE_POSITION_CHANGED;
        if ((input.state.position - remainingPath.front().position).length() > 0.5) return UPDATE_POSITION_CHANGED;

        if ((input.state.velocity - remainingPath.front().velocity).length() > 0.5) return UPDATE_POSITION_CHANGED;
    }

    // Check if the target position has changed while the robot is still
    if (remainingPath.empty() && !PositionControlUtils::positionWithinTolerance(input.targetPos, input.state.position)) return UPDATE_TARGET_REACHED;
    for (int i = 0; i < static_cast<int>(remainingPath.size()); i++) {
        if (!collisionDetector.doesCollideWithMovingObjects(remainingPath[i], input.robotId, input.avoidObjects, i)) continue;
        return UPDATE_COLLISION_DETECTED;
    }
    return DONT_UPDATE;
}

PathTracking::PathTracking(const CollisionDetector& collisionDetector) : collisionDetector(collisionDetector) {}

}  // namespace rtt::ai::control